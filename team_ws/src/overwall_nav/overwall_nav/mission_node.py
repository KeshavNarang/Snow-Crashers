"""ROS node: the over-the-wall mission (logic and explanation in mission.py).

    subscribe  /skybeta/state                  JSON from flight_interface
    subscribe  /visual_slam/tracking/odometry  heading at the start
    subscribe  /overwall/front                 JSON from stereo_depth
    publish    /skybeta/setpoint/pose, /skybeta/setpoint/twist

Never /fmu/in/*, never arms, never changes mode.
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

from overwall_nav.mission import Mission, Params, yaw_from_quaternion


class OverWallNode(Node):

    def __init__(self):
        super().__init__('overwall_mission')
        defaults = Params()
        names = list(vars(defaults))
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
        self.mission = Mission(Params(**{n: self.get_parameter(n).value for n in names}))
        self.state = self.odom = self.front = None
        self.pose_pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.create_subscription(String, '/skybeta/state', self.state_cb, 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_cb, 10)
        self.create_subscription(String, '/overwall/front', self.front_cb, 10)
        # PX4's heading, read only (as flight_interface does): the heading the mission holds.
        version = int(getattr(VehicleLocalPosition, 'MESSAGE_VERSION', 0))
        topic = '/fmu/out/vehicle_local_position' + (f'_v{version}' if version else '')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.px4_heading = None
        self.create_subscription(VehicleLocalPosition, topic, self.local_position_cb, px4_qos)
        self.create_timer(0.1, self.tick)
        p = self.mission.p
        wall = f'wall {p.wall_height_m:.2f} m tall' if p.wall_height_m > 0 else 'wall height from depth'
        self.get_logger().info(
            f'overwall_mission ready: cruise {p.cruise_height_m:.2f} m, stop {p.stop_distance_m:.2f} m from the '
            f'wall, {wall}, max height {p.max_height_m:.1f} m, max forward {p.max_forward_m:.1f} m.\n'
            'Pilot: arm with the mode switch on Altitude, then switch to Position on the ground.')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        state['received'] = self.now_s()
        self.state = state

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom = {'position': (float(p.x), float(p.y), float(p.z)),
                     'yaw': yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
                     'received': self.now_s()}

    def local_position_cb(self, msg):
        heading = float(msg.heading)
        self.px4_heading = (heading, self.now_s()) if math.isfinite(heading) else None

    def front_cb(self, msg):
        try:
            front = json.loads(msg.data)
        except ValueError:
            return
        front['received'] = self.now_s()
        self.front = front

    def tick(self):
        now = self.now_s()
        if self.odom is not None:
            fresh = self.px4_heading is not None and now - self.px4_heading[1] < 1.0
            self.odom['px4_yaw'] = yaw_enu_from_ned(self.px4_heading[0]) if fresh else None
        command = self.mission.step(now, self.state, self.odom, self.front)
        for line in self.mission.log:
            self.get_logger().info(f'[{self.mission.phase}] {line}')
        self.mission.log.clear()
        if command is None:
            return
        kind, (x, y, z) = command
        stamp = self.get_clock().now().to_msg()
        if kind == 'pose':
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = stamp, 'odom'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
            yaw = self.mission.hold_yaw
            if yaw is None:
                msg.pose.orientation.w = 0.0      # all-zero quaternion: do not command yaw
            else:                                 # hold the start heading (ENU yaw about +z)
                msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            self.pose_pub.publish(msg)
        else:
            msg = TwistStamped()
            msg.header.stamp, msg.header.frame_id = stamp, 'odom'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(x), float(y), float(z)
            msg.twist.angular.z = 0.0
            self.twist_pub.publish(msg)


def yaw_enu_from_ned(heading_ned):
    """PX4 heading (NED, from north towards east) -> ENU yaw (from east towards north)."""
    return math.atan2(math.sin(math.pi / 2 - heading_ned), math.cos(math.pi / 2 - heading_ned))


def main(args=None):
    rclpy.init(args=args)
    node = OverWallNode()
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
