"""Depth and a camera image for goalnav, from the OAK-D-S2's mono pair (drone only).

The skybeta stack runs the camera with no on-board depth and no colour camera: it
publishes only ``/oak/{left,right}/image_mono`` (1024x640 at 30 Hz, raw) and their
CameraInfo, for cuVSLAM. This node does the stereo matching on the Jetson with
overwall_nav's ``StereoRig`` (rectify both at ``scale``, OpenCV SGBM, points in the left
camera's frame; tuned on skybeta-03, see agents/logs/2026-11-07-depth-bottleneck.md) and
publishes, for each pair it processes (at most ``rate_hz``):

- ``/goalnav/stereo/points``   PointCloud2, metres in the left camera's optical frame
                               (x right, y down, z forward), stamped with the images;
- ``/goalnav/stereo/left``     the rectified left image at ``scale`` (mono8), and
- ``/goalnav/stereo/left_info`` its CameraInfo: K of the rectified image, R the
                               rectification (X_rectified = R X_left).

It is the only node that takes the full-size 30 Hz images (best effort, depth 2): a
second Python subscriber to them drops frames and loads the cores cuVSLAM runs on. It
never commands the drone. Calibration: both CameraInfo and the TF from the left to the
right optical frame (the stereo extrinsics cuVSLAM uses).
"""
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from tf2_ros import Buffer, TransformListener

from overwall_nav.stereo import StereoRig, extrinsics_from_tf

PARAMS = {'left_image': '/oak/left/image_mono', 'right_image': '/oak/right/image_mono',
          'left_info': '/oak/left/camera_info', 'right_info': '/oak/right/camera_info',
          'points': '/goalnav/stereo/points', 'image': '/goalnav/stereo/left',
          'image_info': '/goalnav/stereo/left_info',
          'rate_hz': 5.0, 'scale': 0.4, 'num_disparities': 96, 'step': 2}


class StereoNode(Node):
    def __init__(self):
        super().__init__('goalnav_stereo')
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAMS}
        self.period = 1. / float(self.p['rate_hz'])
        self.bridge = CvBridge()
        self.info, self.rig, self.last_done = {}, None, 0.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.p['left_info'], lambda m: self.info.setdefault('left', m), 1)
        self.create_subscription(CameraInfo, self.p['right_info'], lambda m: self.info.setdefault('right', m), 1)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
        left = message_filters.Subscriber(self, Image, self.p['left_image'], qos_profile=qos)
        right = message_filters.Subscriber(self, Image, self.p['right_image'], qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer([left, right], queue_size=4, slop=0.01)
        self.sync.registerCallback(self._pair)
        self.points_pub = self.create_publisher(PointCloud2, self.p['points'], 5)
        self.image_pub = self.create_publisher(Image, self.p['image'], 5)
        self.info_pub = self.create_publisher(CameraInfo, self.p['image_info'], 5)

    def _build_rig(self):
        if 'left' not in self.info or 'right' not in self.info:
            return None
        left, right = self.info['left'], self.info['right']
        try:
            tf = self.tf_buffer.lookup_transform(left.header.frame_id, right.header.frame_id, Time())
        except Exception:   # noqa: BLE001 - the static TF arrives a moment after start-up
            return None
        tr, q = tf.transform.translation, tf.transform.rotation
        r, t = extrinsics_from_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        rig = StereoRig(np.array(left.k), np.array(right.k), (left.width, left.height), r, t,
                        d_left=np.array(left.d) if len(left.d) else None,
                        d_right=np.array(right.d) if len(right.d) else None, scale=float(self.p['scale']),
                        num_disparities=int(self.p['num_disparities']), step=int(self.p['step']))
        self.get_logger().info(f'calibration: f {rig.f:.1f} px at scale {rig.scale}, baseline '
                               f'{rig.baseline * 100:.1f} cm, range {rig.min_range:.2f}-{rig.max_range:.1f} m')
        return rig

    def _pair(self, left_msg, right_msg):
        now = time.monotonic()
        if now - self.last_done < self.period:
            return
        if self.rig is None:
            self.rig = self._build_rig()
            if self.rig is None:
                return
        self.last_done = now
        left = self.bridge.imgmsg_to_cv2(left_msg, 'mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')
        points, _ = self.rig.points(left, right)
        header = left_msg.header
        self.points_pub.publish(create_cloud_xyz32(header, points.astype(np.float32)))
        small = cv2.resize(left, self.rig.size, interpolation=cv2.INTER_AREA)
        rectified = cv2.remap(small, *self.rig.maps[0], cv2.INTER_LINEAR)
        image = self.bridge.cv2_to_imgmsg(rectified, 'mono8')
        image.header = header
        self.image_pub.publish(image)
        self.info_pub.publish(CameraInfo(header=header, width=self.rig.size[0], height=self.rig.size[1],
                                         k=[self.rig.f, 0., self.rig.cx, 0., self.rig.f, self.rig.cy, 0., 0., 1.],
                                         r=[float(v) for v in self.rig.r1.ravel()]))


def main(args=None):
    rclpy.init(args=args)
    node = StereoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
