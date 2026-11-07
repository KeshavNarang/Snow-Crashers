"""ROS node: up 2 m, forward 2 m, land (mission.py has the logic and the explanation).

The skybeta contract, exactly as competitor_example/square_mission.py uses it:

    subscribe  /skybeta/state                    JSON: px4_link, armed, landed, offboard_active, local_enu
    subscribe  /visual_slam/tracking/odometry    cuVSLAM pose (ENU, odom): our heading
    publish    /skybeta/setpoint/pose            where to be (ENU, odom); floored at 0.3 m
    publish    /skybeta/setpoint/twist           how fast to move (ENU); expires after 1 s

Never /fmu/in/*, never arms, never changes mode: skybeta_flight_interface owns all of that,
and the safety pilot arms and can take over at any time.
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from snowcrashers_nav.mission import Mission, Params, yaw_from_quaternion


class UpForwardLandNode(Node):

    def __init__(self, **kwargs):
        super().__init__('up_forward_land', **kwargs)
        defaults = Params()
        names = [n for n in vars(defaults)]
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
        self.mission = Mission(Params(**{n: self.get_parameter(n).value for n in names}))
        self.state = None
        self.odom = None
        self.pose_pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.create_subscription(String, '/skybeta/state', self.state_cb, 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_cb, 10)
        self.create_timer(0.1, self.tick)
        p = self.mission.p
        self.get_logger().info(
            f'up_forward_land ready: up {p.height_m:.1f} m, forward {p.forward_m:.1f} m, land '
            f'(final descent {p.descent_slow_mps:.2f} m/s). Publishing setpoints at 10 Hz.\n'
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

    def tick(self):
        command = self.mission.step(self.now_s(), self.state, self.odom)
        for line in self.mission.log:
            self.get_logger().info(f'[{self.mission.phase}] {line}')
        self.mission.log.clear()
        if command is None:
            return
        kind, (x, y, z) = command
        if kind == 'pose':
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
            msg.pose.orientation.w = 0.0          # all-zero quaternion: do not command yaw
            self.pose_pub.publish(msg)
        else:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(x), float(y), float(z)
            msg.twist.angular.z = 0.0             # no yaw rate
            self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UpForwardLandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                            # Ctrl-C or the stack stopping (as the organizers' example)
    finally:
        try:                            # the launch's second Ctrl-C can land in here (Humble)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass
