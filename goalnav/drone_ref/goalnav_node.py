"""ROS 2 adapter for the goal navigator (``drone_ref.goalnav``).

Subscribes: PX4 local position, attitude, status and land detector; the
OAK-D depth and RGB images and their CameraInfo; PX4's distance sensor
(the downward VL53L1X).
Publishes Offboard velocity setpoints and vehicle commands; status JSON on
``/goalnav/status``. ``/goalnav/start`` (std_srvs/Trigger) is the one start
command. The landing target is found in the OAK-D's RGB image
(``goalnav.pad_detect``, from a reference picture of the mat).

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
from px4_msgs.msg import (DistanceSensor, OffboardControlMode, TrajectorySetpoint, VehicleAttitude,
                          VehicleCommand, VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_ref.goalnav.geometry import (camera_to_odom, depth_points, enu_to_ned, ned_to_enu, optical_to_body,
                                        px4_attitude_enu, yaw_enu_to_ned)
from drone_ref.goalnav.floor_line import dark_floor_points
from drone_ref.goalnav.pad_detect import PadDetector
from drone_ref.goalnav.mission import GoalMission, Telemetry
from drone_ref.goalnav import resource
from drone_ref.goalnav.profiles import load as load_profile

OFFBOARD = 14
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
        self.position = self.attitude = self.status = self.land = None
        self.camera = None
        qos = qos_profile_sensor_data
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._position, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._attitude, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._status, qos)
        self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self._land, qos)
        self.create_subscription(Image, p['depth_topic'], self._depth, qos)
        self.create_subscription(CameraInfo, p['camera_info_topic'], self._info, qos)
        self.create_subscription(DistanceSensor, p['range_topic'], self._range, qos)
        self.create_subscription(Image, p['rgb_topic'], self._rgb, qos)
        self.create_subscription(CameraInfo, p['rgb_info_topic'], self._rgb_info, qos)
        self.pad_detector = PadDetector()
        self.rgb_camera = None
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.status_pub = self.create_publisher(String, '/goalnav/status', 10)
        self.create_service(Trigger, '/goalnav/start', self._start)
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

    def _status(self, msg):
        self.status = msg

    def _land(self, msg):
        self.land = msg

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
        if self.position is None or self.attitude is None or self.status is None:
            return None
        m = self.position
        return Telemetry(t=self._now(), position=ned_to_enu((m.x, m.y, m.z)),
                         velocity=ned_to_enu((m.vx, m.vy, m.vz)), rotation=px4_attitude_enu(self.attitude.q),
                         armed=self.status.arming_state == 2, offboard=self.status.nav_state == OFFBOARD,
                         landed=bool(self.land.landed) if self.land is not None else True,
                         preflight_ok=bool(getattr(self.status, 'pre_flight_checks_pass', True)),
                         valid=bool(m.xy_valid and m.z_valid and m.v_xy_valid))

    # -- outputs -----------------------------------------------------------------------
    def _start(self, request, response):
        telemetry = self._telemetry()
        if telemetry is None:
            response.success, response.message = False, 'no PX4 telemetry yet'
        else:
            response.success, response.message = self.mission.start(telemetry)
        return response

    def _tick(self):
        telemetry = self._telemetry()
        if telemetry is None:
            return
        setpoint, commands = self.mission.step(telemetry, self.dt)
        us = int(self._now() * 1e6)
        if setpoint is not None:
            nan = float('nan')
            full = setpoint.position is not None
            self.mode_pub.publish(OffboardControlMode(timestamp=us, position=full, velocity=True, acceleration=full))
            unset = np.array([nan, nan, nan], np.float32)
            self.setpoint_pub.publish(TrajectorySetpoint(
                timestamp=us, position=enu_to_ned(setpoint.position).astype(np.float32) if full else unset,
                velocity=enu_to_ned(setpoint.velocity).astype(np.float32),
                acceleration=enu_to_ned(setpoint.acceleration).astype(np.float32)
                if setpoint.acceleration is not None else unset,
                yaw=float(yaw_enu_to_ned(setpoint.yaw)), yawspeed=nan))
        for command, param1, param2 in commands:
            self.command_pub.publish(VehicleCommand(timestamp=us, command=command, param1=param1, param2=param2,
                                                    target_system=1, target_component=1, source_system=1,
                                                    source_component=1, from_external=True))

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
