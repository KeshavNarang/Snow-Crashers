"""ROS 2 adapter for the goal navigator (``drone_ref.goalnav``).

Subscribes: PX4 local position (also the downward rangefinder's reading,
``dist_bottom``: PX4 v1.16 does not publish distance_sensor over DDS) and
attitude, read only; ``/skybeta/state`` (armed, Offboard, landed, from
skybeta's flight_interface); the OAK-D's own stereo depth image (computed on
the camera; goalnav does no depth estimation of its own) and one camera
image for the floor tape and the landing pad (the pad needs a colour one).
Publishes velocity setpoints on ``/skybeta/setpoint/twist``
(``goalnav.skybeta``) and status JSON on ``/goalnav/status``; never anything
on ``/fmu/in/*``. It streams a zero twist from start-up, so flight_interface
may enter Offboard when the pilot arms and then switches to Position; the
mission starts (takes off) as soon as Offboard is active. The landing target
is found in the colour image (``goalnav.pad_detect``, from a reference picture
of the mat), or in ``pad_image_topic`` if that names another one. The colour
image is undistorted with its CameraInfo before use; the depth image is
already rectified (pinhole).

Depth points are placed with the pose received nearest the image's capture
stamp (both on the ROS clock), so latency does not smear the map. Each image's
camera mount comes from the TF (``base_frame`` -> the message's frame, the
driver's factory calibration); ``camera_offset`` and ``camera_pitch_deg`` are
the fallback when no TF is published (the simulator).
"""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String

from drone_ref.goalnav import skybeta
from drone_ref.goalnav.geometry import (camera_to_odom, depth_points, ned_to_enu, optical_to_body,
                                        px4_attitude_enu, quaternion_matrix)
from drone_ref.goalnav.floor_line import dark_floor_points
from drone_ref.goalnav.pad_detect import PadDetector
from drone_ref.goalnav.mission import GoalMission, Telemetry
from drone_ref.goalnav import resource
from drone_ref.goalnav.profiles import load as load_profile

PARAMS = {
    'goals_file': 'config/goals.yaml',
    # On-camera stereo depth (team_ws/camera/oakd_vslam.yaml): 16UC1 mm or 32FC1 m.
    'depth_topic': '/oak/stereo/image_raw', 'depth_info_topic': '/oak/stereo/camera_info',
    'depth_tile': 8,                                       # nearest return of each tile x tile block
    'points_topic': '',                                    # optional PointCloud2 source instead
    # The image for the floor tape and the pad. The team camera config runs no colour camera (it
    # cost cuVSLAM ~7 Hz, agents/logs/2026-11-07-goalnav-hardware.md), so: the left mono image; the
    # pad needs colour (set /oak/rgb/image_raw and /oak/rgb/camera_info when it runs).
    'image_topic': '/oak/left/image_mono', 'image_info_topic': '/oak/left/camera_info',
    'pad_image_topic': '', 'pad_info_topic': '',          # empty: the pad is found in image_topic
    # Fallback mount when the TF has none. skybeta-03's TF (base_link -> oak_left_camera_optical_frame):
    # (0.106, 0.038, 0.031) m, axis 27.6 deg down, 0.7 deg yaw left, 0.5 deg roll.
    'base_frame': 'base_link',
    'camera_offset': [0.105, 0.038, 0.030], 'camera_pitch_deg': 27.6, 'max_points': 6000,
    'min_depth': 0.25, 'max_depth': 6.0, 'control_hz': 50.,
    'profile': 'safe', 'profiles_file': 'config/goalnav_profiles.yaml',
    'tuning': '',            # YAML dict of overrides on the profile, e.g. "{max_speed: 0.7}"
}


def depth_image_m(msg):
    """A depth Image as metres (float32), 0 where there is no return; 16UC1
    (millimetres, the OAK driver's) or 32FC1 (metres). None otherwise."""
    if msg.encoding in ('16UC1', 'mono16'):
        raw = np.frombuffer(bytes(msg.data), dtype='>u2' if msg.is_bigendian else '<u2')
        return raw.reshape(msg.height, msg.step // 2)[:, :msg.width].astype(np.float32) * 1e-3
    if msg.encoding == '32FC1':
        raw = np.frombuffer(bytes(msg.data), dtype='>f4' if msg.is_bigendian else '<f4')
        return np.nan_to_num(raw.reshape(msg.height, msg.step // 4)[:, :msg.width], nan=0., posinf=0.)
    return None


def _stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class GoalNavNode(Node):
    def __init__(self):
        super().__init__('drone_ref_goalnav')
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMS}
        self.p = p
        path = Path(p['goals_file'])
        if not path.is_absolute():
            path = resource(path)
        spec = yaml.safe_load(path.read_text())
        profiles = Path(p['profiles_file'])
        if not profiles.is_absolute():
            profiles = resource(profiles)
        configs = load_profile(p['profile'], profiles, yaml.safe_load(p['tuning']) if p['tuning'] else None)
        self.mission = GoalMission(spec, *configs)
        self.get_logger().info(f"goalnav profile {p['profile']}: {configs}")
        self.history = deque(maxlen=200)          # (ros time s, position ENU, rotation)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mounts = {}                          # frame id -> (offset, optical -> body rotation)
        self.position = self.attitude = None
        self.flight = (False, False, None)        # (armed, offboard, landed) from /skybeta/state
        qos = qos_profile_sensor_data
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._position, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._attitude, qos)
        self.create_subscription(String, skybeta.STATE_TOPIC, self._state, 10)
        if p['depth_topic']:
            self.create_subscription(Image, p['depth_topic'], self._depth, qos)
            self.create_subscription(CameraInfo, p['depth_info_topic'], self._depth_info, qos)
        if p['points_topic']:
            self.create_subscription(PointCloud2, p['points_topic'], self._points, qos)
        self.create_subscription(Image, p['image_topic'], self._image, qos)
        self.create_subscription(CameraInfo, p['image_info_topic'], self._image_info, qos)
        if p['pad_image_topic']:
            self.create_subscription(Image, p['pad_image_topic'], self._pad_image, qos)
            self.create_subscription(CameraInfo, p['pad_info_topic'], self._pad_info, qos)
        self.pad_detector = PadDetector()
        self.image_camera = self.pad_camera = self.depth_camera = None   # (fx, fy, cx, cy, R, undistort maps)
        self.floor_cells = None                          # the last cloud's floor, for the tape check
        self.twist_pub = self.create_publisher(TwistStamped, skybeta.TWIST_TOPIC, 10)
        self.status_pub = self.create_publisher(String, '/goalnav/status', 10)
        self.dt = 1. / float(p['control_hz'])
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.2, self._publish_status)

    # -- inputs ----------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _attitude(self, msg):
        self.attitude = msg
        self._remember()

    def _remember(self):
        if self.position is None or self.attitude is None:
            return
        m = self.position
        self.history.append((self._now(), ned_to_enu((m.x, m.y, m.z)), px4_attitude_enu(self.attitude.q)))

    def _state(self, msg):
        self.flight = skybeta.parse_state(msg.data)

    def _pose_at(self, when):
        if not self.history:
            return None
        best = min(self.history, key=lambda h: abs(h[0] - when))
        return best if abs(best[0] - when) < 0.1 else None

    def _mount(self, frame_id):
        """(offset, optical -> body rotation) of the camera whose optical frame is
        ``frame_id``: from the TF once it has arrived, else the parameters."""
        if frame_id in self.mounts:
            return self.mounts[frame_id]
        try:
            tf = self.tf_buffer.lookup_transform(self.p['base_frame'], frame_id, Time()).transform
        except Exception:   # noqa: BLE001 - no TF (simulator), or not received yet
            return (np.asarray(self.p['camera_offset'], float),
                    optical_to_body(math.radians(float(self.p['camera_pitch_deg']))))
        t, q = tf.translation, tf.rotation
        mount = np.array([t.x, t.y, t.z]), quaternion_matrix((q.w, q.x, q.y, q.z))
        down = math.degrees(math.asin(-mount[1][2, 2]))
        self.get_logger().info(f'{frame_id} mount from TF: offset {np.round(mount[0], 3)} m, {down:.1f} deg down')
        self.mounts[frame_id] = mount
        return mount

    def _camera_pose(self, pose, frame_id, rectification=None):
        """(origin, optical -> odom rotation) of the camera at ``pose``; a
        rectified image's frame is turned by R (X_rectified = R X_camera)."""
        offset, mount = self._mount(frame_id)
        rotation = pose[2] @ mount
        if rectification is not None:
            rotation = rotation @ rectification.T
        return pose[1] + pose[2] @ offset, rotation

    def _depth(self, msg):
        """The camera's depth image (its own optical frame) into the live map."""
        if self.depth_camera is None:
            return
        depth = depth_image_m(msg)
        if depth is None:
            return
        fx, fy, cx, cy = self.depth_camera[:4]
        points = depth_points(depth, fx, fy, cx, cy, tile=int(self.p['depth_tile']),
                              min_depth=float(self.p['min_depth']), max_depth=float(self.p['max_depth']))
        self._add_points(points, msg.header)

    def _depth_info(self, msg):
        self.depth_camera = self._camera_info(msg, undistort=False)

    def _points(self, msg):
        """A PointCloud2 (in its header's optical frame) into the live map."""
        if not msg.width:
            return
        points = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :3]
        self._add_points(points, msg.header)

    def _add_points(self, points, header):
        pose = self._pose_at(_stamp_s(header))
        if pose is None or not len(points):
            return
        distance = np.linalg.norm(points, axis=1)
        points = points[(distance > float(self.p['min_depth'])) & (distance < float(self.p['max_depth']))]
        if len(points) > int(self.p['max_points']):
            points = points[np.linspace(0, len(points) - 1, int(self.p['max_points'])).astype(int)]
        origin, rotation = self._camera_pose(pose, header.frame_id)
        world = points.astype(float) @ rotation.T + origin
        self.mission.add_depth(origin, world)
        if self.mission.floor is not None:
            floor = world[np.abs(world[:, 2] - self.mission.floor.floor_z) < 0.06]
            self.floor_cells = set(map(tuple, np.floor(floor[:, :2] / 0.1).astype(int)))

    def _position(self, msg):
        self.position = msg
        self._remember()
        telemetry = self._telemetry()
        if telemetry is not None:      # The downward rangefinder, as PX4 reports it.
            self.mission.add_range(telemetry, float(msg.dist_bottom) if msg.dist_bottom_valid else None)

    @staticmethod
    def _camera_info(msg, undistort=True):
        """(fx, fy, cx, cy, R, undistortion maps or None); the maps are made
        once per calibration, only when the image is distorted."""
        k = np.asarray(msg.k, float)
        r = np.asarray(msg.r, float).reshape(3, 3) if len(msg.r) == 9 and any(msg.r) else np.eye(3)
        maps = None
        if undistort and len(msg.d) and np.any(np.asarray(msg.d, float)):
            import cv2      # only for a distorted image (not in the simulator)
            maps = cv2.initUndistortRectifyMap(k.reshape(3, 3), np.asarray(msg.d, float), None, k.reshape(3, 3),
                                               (msg.width, msg.height), cv2.CV_16SC2)
        return k[0], k[4], k[2], k[5], r, maps

    def _image_info(self, msg):
        if self.image_camera is None or self.image_camera[:4] != self._camera_info(msg, False)[:4]:
            self.image_camera = self._camera_info(msg)

    def _pad_info(self, msg):
        if self.pad_camera is None or self.pad_camera[:4] != self._camera_info(msg, False)[:4]:
            self.pad_camera = self._camera_info(msg)

    def _frame(self, msg, camera):
        """(image array, intrinsics, camera origin, rotation) or None."""
        pose = self._pose_at(_stamp_s(msg.header))
        if camera is None or pose is None or self.mission.floor is None:
            return None
        channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
        image = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, channels)
        image = image[..., ::-1] if msg.encoding == 'bgr8' else image
        if camera[5] is not None:
            import cv2
            image = cv2.remap(np.ascontiguousarray(image), *camera[5], cv2.INTER_LINEAR)
            image = image if image.ndim == 3 else image[..., None]
        origin, rotation = self._camera_pose(pose, msg.header.frame_id, camera[4])
        return (image if channels == 3 else image[..., 0]), camera[:4], origin, rotation

    def _wants_pad(self):
        m = self.mission
        return m.route is not None and any(g.kind == 'pad' for g in m.route.upcoming(4))

    def _image(self, msg):
        """The floor tape (CP1) while it matters, and the pad near the end
        (unless ``pad_image_topic`` gives another image for it)."""
        m = self.mission
        line = m.route is not None and m.wants_line()
        pad = not self.p['pad_image_topic'] and self._wants_pad()
        if not (line or pad):
            return
        frame = self._frame(msg, self.image_camera)
        if frame is None:
            return
        image, k, origin, rotation = frame
        if line:
            m.add_line(self._confirmed_floor(dark_floor_points(image, *k, origin, rotation, m.floor.floor_z)))
        if pad:
            self._find_pad(frame)

    def _pad_image(self, msg):
        """The landing target in ``pad_image_topic``, once the route is near its end."""
        if self._wants_pad():
            frame = self._frame(msg, self.pad_camera)
            if frame is not None:
                self._find_pad(frame)

    def _find_pad(self, frame):
        m = self.mission
        found = self.pad_detector.detect(frame[0], *frame[1], frame[2], frame[3], m.floor.floor_z)
        if found is not None:
            m.add_pad(found)

    def _confirmed_floor(self, points):
        """Keep tape points where the last depth frame saw floor (a dark person
        beyond the net, cast onto the floor plane, is not on it)."""
        if not len(points) or not self.floor_cells:
            return points[:0]
        cells = np.floor(points / 0.1).astype(int)
        keep = np.array([tuple(c) in self.floor_cells for c in cells], bool)
        return points[keep]

    def _telemetry(self):
        if self.position is None or self.attitude is None:
            return None
        m = self.position
        armed, offboard, landed = self.flight
        return Telemetry(t=self._now(), position=ned_to_enu((m.x, m.y, m.z)),
                         velocity=ned_to_enu((m.vx, m.vy, m.vz)), rotation=px4_attitude_enu(self.attitude.q),
                         armed=armed, offboard=offboard, landed=True if landed is None else bool(landed),
                         valid=bool(m.xy_valid and m.z_valid and m.v_xy_valid))

    # -- outputs -----------------------------------------------------------------------
    def _tick(self):
        """One twist per control tick, zero when there is nothing to fly (it
        is also the stream flight_interface needs before it will enter
        Offboard). The mission starts when Offboard is active."""
        telemetry = self._telemetry()
        setpoint = None
        if telemetry is not None:
            if self.mission.state == 'IDLE' and telemetry.armed and telemetry.offboard:
                ok, why = self.mission.start(telemetry)
                self.get_logger().info(f'Offboard active: mission start: {why}')
            setpoint = self.mission.step(telemetry, self.dt)
        velocity, yaw_rate = (np.zeros(3), 0.) if setpoint is None else skybeta.twist(setpoint, telemetry)
        msg = skybeta.stamp(TwistStamped(), self._now())
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(v) for v in velocity)
        msg.twist.angular.z = yaw_rate
        self.twist_pub.publish(msg)

    def _publish_status(self):
        m = self.mission
        route = m.route
        out = {'state': m.state, 'reason': m.status.reason,
               'goal': route.current.id if route and route.current else None,
               'speed_limit': round(m.speed, 2), 'free_ahead': round(m.free_ahead, 2),
               'heading_correction_deg': round(math.degrees(route.heading_correction), 2) if route else None,
               'floor_z': round(m.floor.floor_z, 3) if m.floor else None,
               'range': m.floor.last if m.floor else None,
               'events': m.status.events[-3:],          # what it decided last (passed goals, phases)
               'pad': [round(float(v), 3) for v in m.route.goals[-1].estimate] if route else None}
        self.status_pub.publish(String(data=json.dumps(out, default=str)))


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
