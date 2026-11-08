"""ROS node: stereo depth from the OAK-D-S2 mono pair -> /overwall/front.

Reads only (never commands the drone), so it is safe to run on the bench by itself:

    ros2 run overwall_nav stereo_depth          # one summary line a second

Subscribes  /oak/left/image_mono, /oak/right/image_mono (best effort: never slows the camera
            pipeline cuVSLAM depends on), /oak/{left,right}/camera_info, TF left -> right.
Publishes   /overwall/front  std_msgs/String JSON:
            cells          [[fi, ui, count], ...] 5 cm cells (forward, up from the camera, level
                           frame) of the points in a drone-wide corridor (perception.py)
            top_edge_deg   elevation of the highest ray covered (-0.5 deg at a 27 deg mount)
            floor          floor-plane fit: camera pitch_deg / roll_deg / height_m, or null
            pitch_deg      the mount pitch used (parameter camera_pitch_deg)
            tf_pitch_deg   the mount pitch the stack's TF claims (for comparison)
            valid, n, proc_ms, age_ms
"""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import message_filters
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from overwall_nav.perception import analyze, bin_points
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, fit_floor, level, quaternion_to_matrix


class StereoDepthNode(Node):

    def __init__(self):
        super().__init__('stereo_depth')
        for name, default in (('left_image', '/oak/left/image_mono'), ('right_image', '/oak/right/image_mono'),
                              ('left_info', '/oak/left/camera_info'), ('right_info', '/oak/right/camera_info'),
                              ('output', '/overwall/front'), ('base_frame', 'base_link'),
                              ('rate_hz', 5.0), ('scale', 0.5), ('num_disparities', 96), ('step', 2),
                              ('camera_pitch_deg', 27.0), ('corridor_m', 0.4), ('swap_lr', False),
                              ('log_period_s', 1.0)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.period = 1.0 / float(g('rate_hz'))
        self.log_period = float(g('log_period_s'))
        self.swap = bool(g('swap_lr'))
        self.pitch = float(g('camera_pitch_deg'))
        self.corridor = float(g('corridor_m'))
        self.base_frame = g('base_frame')
        self.rig_args = dict(scale=float(g('scale')), num_disparities=int(g('num_disparities')), step=int(g('step')))
        self.bridge = CvBridge()
        self.info = {}
        self.rig = None
        self.top_edge = 0.0
        self.tf_pitch = None
        self.rig_wait_since = time.monotonic()
        self.last_done = self.last_log = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(String, g('output'), 10)
        self.create_subscription(CameraInfo, g('left_info'), lambda m: self.info.setdefault('left', m), 1)
        self.create_subscription(CameraInfo, g('right_info'), lambda m: self.info.setdefault('right', m), 1)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
        left = message_filters.Subscriber(self, Image, g('left_image'), qos_profile=qos)
        right = message_filters.Subscriber(self, Image, g('right_image'), qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer([left, right], queue_size=4, slop=0.01)
        self.sync.registerCallback(self.on_pair)
        self.get_logger().info(f'stereo_depth up: {g("left_image")} + {g("right_image")} -> {g("output")} at '
                               f'<= {g("rate_hz")} Hz, camera mount pitch {self.pitch:.1f} deg down')

    # -- calibration -----------------------------------------------------------
    def build_rig(self):
        if 'left' not in self.info or 'right' not in self.info:
            return None
        li, ri = self.info['left'], self.info['right']
        if self.swap:
            li, ri = ri, li
        source = 'TF'
        try:
            tf = self.tf_buffer.lookup_transform(li.header.frame_id, ri.header.frame_id, Time())
            tr, q = tf.transform.translation, tf.transform.rotation
            r, t = extrinsics_from_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        except Exception as exc:  # noqa: BLE001
            if time.monotonic() - self.rig_wait_since < 5.0:
                return None
            # No TF after 5 s: baseline from CameraInfo P[3] (one of the two carries -fx*B), no rotation.
            px = [abs(float(m.p[3])) / float(m.p[0]) for m in (li, ri) if float(m.p[0]) > 0]
            baseline = max(px) if px and max(px) > 0.01 else 0.075
            r, t = np.eye(3), np.array([[-baseline], [0.0], [0.0]])
            source = f'CameraInfo baseline {baseline:.3f} m (no TF: {exc})'
        try:   # what the stack's TF says about the mount, to compare with the real pitch
            tf = self.tf_buffer.lookup_transform(self.base_frame, li.header.frame_id, Time())
            q = tf.transform.rotation
            z_axis = quaternion_to_matrix(q.x, q.y, q.z, q.w)[:, 2]     # optical axis in base_link (FLU)
            self.tf_pitch = round(math.degrees(math.asin(max(-1.0, min(1.0, -z_axis[2])))), 1)
        except Exception:  # noqa: BLE001
            self.tf_pitch = None
        rig = StereoRig(np.array(li.k), np.array(ri.k), (li.width, li.height), r, t,
                        d_left=np.array(li.d) if len(li.d) else None,
                        d_right=np.array(ri.d) if len(ri.d) else None, **self.rig_args)
        self.top_edge = rig.top_elevation_deg(self.pitch)
        self.get_logger().info(
            f'calibration from {source}: f {rig.f:.1f} px (scaled), baseline {rig.baseline * 100:.1f} cm, '
            f'range {rig.min_range:.2f}-{rig.max_range:.1f} m; mount {self.pitch:.1f} deg down (TF says '
            f'{self.tf_pitch if self.tf_pitch is not None else "?"}), highest ray {self.top_edge:.1f} deg'
            f'{", left/right SWAPPED" if self.swap else ""}')
        return rig

    # -- per pair ------------------------------------------------------------------
    def on_pair(self, left_msg, right_msg):
        now = time.monotonic()
        if now - self.last_done < self.period:
            return
        if self.rig is None:
            self.rig = self.build_rig()
            if self.rig is None:
                return
        self.last_done = now
        if self.swap:
            left_msg, right_msg = right_msg, left_msg
        t0 = time.perf_counter()
        left = self.bridge.imgmsg_to_cv2(left_msg, 'mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')
        pts, valid = self.rig.points(left, right)
        cells = bin_points(level(pts, self.pitch), self.corridor)
        floor = fit_floor(pts, pitch_deg=self.pitch)
        reading = {'cells': cells, 'top_edge_deg': round(self.top_edge, 2), 'floor': floor,
                   'pitch_deg': self.pitch, 'tf_pitch_deg': self.tf_pitch,
                   'valid': round(valid, 3), 'n': int(len(pts)),
                   'proc_ms': round((time.perf_counter() - t0) * 1e3, 1)}
        reading['age_ms'] = round((self.get_clock().now() - Time.from_msg(left_msg.header.stamp)).nanoseconds * 1e-6, 1)
        self.pub.publish(String(data=json.dumps(reading)))
        if now - self.last_log >= self.log_period:
            self.last_log = now
            a = analyze(cells, floor['height_m'] if floor else None, self.top_edge)
            fl = (f'camera {floor["pitch_deg"]:.1f} deg down, {floor["height_m"]:.2f} m above the floor '
                  f'({floor["inliers"] * 100:.0f}% of points)' if floor else 'no floor fit')
            self.get_logger().info(
                f'front {"%.2f m" % a["front"] if a["front"] is not None else "clear"} | floor seen to '
                f'{"%.1f m" % a["floor_free"] if a["floor_free"] is not None else "?"} | {fl} | '
                f'{valid * 100:.0f}% matched, {reading["proc_ms"]:.0f} ms, image {reading["age_ms"]:.0f} ms old')


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass
