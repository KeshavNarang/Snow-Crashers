"""ROS node: the landing mat in the colour image -> /overwall/pad (logic in pad.py).

Reads only (never commands the drone):

    ros2 run overwall_nav pad_detect            # one summary line a second

Subscribes  /oak/rgb/image_raw (best effort), /oak/rgb/camera_info. The stack's stock camera
            config publishes no colour: it needs the team camera config with the colour camera on
            (agents/logs/2026-11-07-overwall-pad.md).
Publishes   /overwall/pad  std_msgs/String JSON, every processed frame (also when nothing is seen,
            so the mission knows the detector runs):
            candidates   [{'centre': [f, l], 'sides': [short, long], 'touches', 'pixels'}, ...]
                         floor metres PER METRE of camera height, camera level frame (pad.py)
            pitch_deg, proc_ms, age_ms
"""
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from overwall_nav.pad import PadFinder


class PadDetectNode(Node):

    def __init__(self):
        super().__init__('pad_detect')
        for name, default in (('image', '/oak/rgb/image_raw'), ('info', '/oak/rgb/camera_info'),
                              ('output', '/overwall/pad'), ('rate_hz', 5.0), ('camera_pitch_deg', 27.0),
                              ('log_period_s', 1.0)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.period = 1.0 / float(g('rate_hz'))
        self.pitch = float(g('camera_pitch_deg'))
        self.log_period = float(g('log_period_s'))
        self.finder = PadFinder()
        self.bridge = CvBridge()
        self.info = None
        self.last_done = self.last_log = 0.0
        self.seen = self.frames = 0
        self.pub = self.create_publisher(String, g('output'), 10)
        self.create_subscription(CameraInfo, g('info'), self.info_cb, 1)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, g('image'), self.on_image, qos)
        self.get_logger().info(f'pad_detect up: {g("image")} -> {g("output")} at <= {g("rate_hz")} Hz, '
                               f'camera pitch {self.pitch:.1f} deg down')

    def info_cb(self, msg):
        if self.info is None:
            self.info = msg
            self.get_logger().info(f'colour camera {msg.width}x{msg.height}, fx {msg.k[0]:.1f}, '
                                   f'distortion {msg.distortion_model} ({len(msg.d)} coefficients)')

    def undistort(self, uv):
        import cv2
        k = np.array(self.info.k, float).reshape(3, 3)
        d = np.array(self.info.d, float)
        if not d.size or not np.any(d):
            return uv
        out = cv2.undistortPoints(uv.reshape(-1, 1, 2).astype(np.float32), k, d, P=k)
        return out.reshape(-1, 2).astype(float)

    def on_image(self, msg):
        start = time.monotonic()
        if self.info is None or start - self.last_done < self.period:
            return
        self.last_done = start
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'image conversion failed: {exc}', throttle_duration_sec=5.0)
            return
        k = np.array(self.info.k, float).reshape(3, 3)
        if image.shape[1] != self.info.width:                  # camera_info for another size: scale it
            s = image.shape[1] / float(self.info.width)
            k = k * np.array([[s, 1, s], [1, s, s], [1, 1, 1]])
        candidates = self.finder.candidates(image, k, self.pitch, undistort=self.undistort)
        proc_ms = (time.monotonic() - start) * 1000.0
        age_ms = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-6
        self.pub.publish(String(data=json.dumps({'candidates': candidates, 'pitch_deg': self.pitch,
                                                 'proc_ms': round(proc_ms, 1), 'age_ms': round(age_ms, 1)})))
        self.frames += 1
        self.seen += bool(candidates)
        if start - self.last_log >= self.log_period:
            self.last_log = start
            best = max(candidates, key=lambda c: c['pixels']) if candidates else None
            self.get_logger().info(
                f'{self.frames} frames, {self.seen} with a blue blob; {proc_ms:.0f} ms, {age_ms:.0f} ms old; '
                + (f"biggest {best['pixels']} px at {best['centre'][0]:.2f}/{best['centre'][1]:.2f} per m height, "
                   f"sides {best['sides'][0]:.2f}/{best['sides'][1]:.2f}{' (cut off)' if best['touches'] else ''}"
                   if best else 'none'))
            self.frames = self.seen = 0


def main(args=None):
    rclpy.init(args=args)
    node = PadDetectNode()
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
