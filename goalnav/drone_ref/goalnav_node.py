"""ROS 2 adapter for the goal navigator (``drone_ref.goalnav``).

Subscribes: PX4 local position and attitude (read only); ``/skybeta/state``
(armed, Offboard, landed, from skybeta's flight_interface); the OAK-D depth
and RGB images and their CameraInfo; PX4's distance sensor (the downward
VL53L1X).
Publishes velocity setpoints on ``/skybeta/setpoint/twist``
(``goalnav.skybeta``) and status JSON on ``/goalnav/status``; never anything
on ``/fmu/in/*``. It streams a zero twist from start-up, so flight_interface
may enter Offboard when the pilot arms and then switches to Position; the
mission starts (takes off) as soon as Offboard is active. The landing target
is found in the OAK-D's RGB image (``goalnav.pad_detect``, from a reference
picture of the mat).

Depth points are placed with the pose received nearest the image's capture
stamp (both on the ROS clock), so latency does not smear the map.
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
from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import DistanceSensor, VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from drone_ref.goalnav import skybeta
from drone_ref.goalnav.geometry import camera_to_odom, depth_points, ned_to_enu, optical_to_body, px4_attitude_enu
from drone_ref.goalnav.floor_line import dark_floor_points
from drone_ref.goalnav.pad_detect import PadDetector
from drone_ref.goalnav.mission import GoalMission, Telemetry
from drone_ref.goalnav import resource
from drone_ref.goalnav.profiles import load as load_profile

PARAMS = {
    'goals_file': 'config/goals.yaml',
    'depth_topic': '/oak/stereo/image_raw', 'camera_info_topic': '/oak/stereo/camera_info',
    'range_topic': '/fmu/out/distance_sensor',
    'rgb_topic': '/oak/rgb/image_raw', 'rgb_info_topic': '/oak/rgb/camera_info',
    'camera_offset': [0.10, 0., 0.], 'camera_pitch_deg': 26.7, 'depth_tile': 8,
    'min_depth': 0.31, 'max_depth': 6.0, 'control_hz': 50.,
    'profile': 'safe', 'profiles_file': 'config/goalnav_profiles.yaml',
    'tuning': '',            # YAML dict of overrides on the profile, e.g. "{max_speed: 0.7}"
}


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
        self.position = self.attitude = None
        self.flight = (False, False, None)        # (armed, offboard, landed) from /skybeta/state
        self.camera = None
        qos = qos_profile_sensor_data
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._position, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._attitude, qos)
        self.create_subscription(String, skybeta.STATE_TOPIC, self._state, 10)
        self.create_subscription(Image, p['depth_topic'], self._depth, qos)
        self.create_subscription(CameraInfo, p['camera_info_topic'], self._info, qos)
        self.create_subscription(DistanceSensor, p['range_topic'], self._range, qos)
        self.create_subscription(Image, p['rgb_topic'], self._rgb, qos)
        self.create_subscription(CameraInfo, p['rgb_info_topic'], self._rgb_info, qos)
        self.pad_detector = PadDetector()
        self.rgb_camera = None
        self.twist_pub = self.create_publisher(TwistStamped, skybeta.TWIST_TOPIC, 10)
        self.status_pub = self.create_publisher(String, '/goalnav/status', 10)
        self.dt = 1. / float(p['control_hz'])
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.2, self._publish_status)

    # -- inputs ----------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _position(self, msg):
        self.position = msg
        self._remember()

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

    def _info(self, msg):
        k = np.asarray(msg.k, float)
        self.camera = (k[0], k[4], k[2], k[5])

    def _pose_at(self, when):
        if not self.history:
            return None
        best = min(self.history, key=lambda h: abs(h[0] - when))
        return best if abs(best[0] - when) < 0.1 else None

    def _depth(self, msg):
        if self.camera is None:
            return
        pose = self._pose_at(_stamp_s(msg.header))
        if pose is None:
            return
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint16 if msg.encoding == '16UC1' else np.float32)
        depth = raw.reshape(msg.height, msg.width).astype(np.float32)
        if msg.encoding == '16UC1':
            depth = depth / 1000.
        p = self.p
        self.last_depth = (depth, pose)
        points = depth_points(depth, *self.camera, tile=int(p['depth_tile']),
                              min_depth=float(p['min_depth']), max_depth=float(p['max_depth']))
        world, origin = camera_to_odom(points, pose[1], pose[2], p['camera_offset'],
                                       math.radians(float(p['camera_pitch_deg'])))
        self.mission.add_depth(origin, world)

    def _range(self, msg):
        telemetry = self._telemetry()
        if telemetry is not None:
            distance = float(msg.current_distance) if msg.signal_quality != 0 else None
            self.mission.add_range(telemetry, distance)

    def _rgb_info(self, msg):
        k = np.asarray(msg.k, float)
        self.rgb_camera = (k[0], k[4], k[2], k[5])

    def _rgb(self, msg):
        """Look for the landing target once the route is near its end."""
        m = self.mission
        if self.rgb_camera is None or m.route is None or m.floor is None or msg.encoding != 'rgb8':
            return
        want_pad = any(g.kind == 'pad' for g in m.route.upcoming(4))
        if not (want_pad or m.wants_line()):
            return
        pose = self._pose_at(_stamp_s(msg.header))
        if pose is None:
            return
        image = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        rotation = pose[2] @ optical_to_body(math.radians(float(self.p['camera_pitch_deg'])))
        origin = pose[1] + pose[2] @ np.asarray(self.p['camera_offset'], float)
        if m.wants_line() and getattr(self, 'last_depth', None) is not None:
            points = dark_floor_points(image, *self.rgb_camera, origin, rotation, m.floor.floor_z)
            m.add_line(self._confirmed_floor(points, m.floor.floor_z))
        if want_pad:
            found = self.pad_detector.detect(image, *self.rgb_camera, origin, rotation, m.floor.floor_z)
            if found is not None:
                m.add_pad(found)

    def _confirmed_floor(self, points, floor_z, tolerance=0.1):
        """Keep floor points the stereo depth also sees as floor (a dark
        person beyond the net, cast onto the floor plane, is not)."""
        if not len(points):
            return points
        depth, pose = self.last_depth
        fx, fy, cx, cy = self.camera
        rotation = pose[2] @ optical_to_body(math.radians(float(self.p['camera_pitch_deg'])))
        origin = pose[1] + pose[2] @ np.asarray(self.p['camera_offset'], float)
        world = np.column_stack([points, np.full(len(points), floor_z)])
        optical = (world - origin) @ rotation          # odom -> optical (rotation is optical -> odom)
        z = optical[:, 2]
        front = z > 0.2
        u = np.round(optical[:, 0] / np.maximum(z, 1e-6) * fx + cx).astype(int)
        v = np.round(optical[:, 1] / np.maximum(z, 1e-6) * fy + cy).astype(int)
        h, w = depth.shape
        inside = front & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        seen = np.zeros(len(points))
        seen[inside] = depth[v[inside], u[inside]]
        agree = inside & (np.abs(seen - z) < tolerance * np.maximum(z, 1.))
        return points[agree]

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
               'range': m.floor.last if m.floor else None}
        self.status_pub.publish(String(data=json.dumps(out, default=str)))


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
