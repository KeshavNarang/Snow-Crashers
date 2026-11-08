"""ROS node: the PS5 controller's messages from the laptop -> /overwall/remote (logic in mission.py).

The laptop (tools/ps5) reaches this over an SSH tunnel, so it listens on 127.0.0.1 only: nothing
new is exposed on the network. One client at a time; one JSON object per line:

    {"seq": 812, "sticks": [forward, left, up], "events": [[id, "land"|"hold"|"resume"|"clear"], ...]}

sticks are -1..1 and already zero unless L1 (the dead-man) is held. Each line is republished as
std_msgs/String on /overwall/remote. This node never commands the drone: the mission decides, and
ignores anything older than remote_stale_s (0.3 s), so a dropped link means no nudges.
"""
import json
import socket
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class RemoteLinkNode(Node):

    def __init__(self):
        super().__init__('remote_link')
        self.declare_parameter('port', 47000)
        self.declare_parameter('output', '/overwall/remote')
        self.port = int(self.get_parameter('port').value)
        self.pub = self.create_publisher(String, str(self.get_parameter('output').value), 10)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', self.port))
        self.server.listen(1)
        threading.Thread(target=self.serve, daemon=True).start()
        self.get_logger().info(f'remote_link up: 127.0.0.1:{self.port} -> /overwall/remote (tools/ps5 on the laptop)')

    def serve(self):
        while rclpy.ok():
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            self.get_logger().info('controller connected')
            conn.settimeout(2.0)
            buf = b''
            try:
                while rclpy.ok():
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue                       # quiet laptop: the mission sees stale input
                    if not chunk:
                        break
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        try:
                            msg = json.loads(line)
                        except ValueError:
                            continue
                        if isinstance(msg, dict):
                            self.pub.publish(String(data=json.dumps(msg)))
            finally:
                conn.close()
                self.get_logger().warn('controller disconnected: no nudges until it reconnects')


def main(args=None):
    rclpy.init(args=args)
    node = RemoteLinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.server.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass
