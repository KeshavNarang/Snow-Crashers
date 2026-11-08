"""Log, once a second, how old cuVSLAM's odometry (and optionally other topics) is on arrival.

One long-lived process, so the measurement does not keep adding DDS participants the way
repeated `ros2` CLI calls do. Run inside skybeta-stack:
    python3 agemon.py [--secs N] [topic:type ...]  > /tmp/agemon.log
Each line: wall time, then per topic "name n=<msgs> med=<ms> max=<ms>" (age = receive time -
header stamp). Read-only.
"""
import argparse
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

DEFAULT = ['/visual_slam/tracking/odometry:nav_msgs/msg/Odometry',
           '/goalnav/stereo/points:sensor_msgs/msg/PointCloud2']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=600)
    ap.add_argument('topics', nargs='*', default=DEFAULT)
    a = ap.parse_args()
    rclpy.init()
    node = Node('agemon')
    ages = {}
    for spec in a.topics:
        name, typ = spec.split(':')
        ages[name] = []

        def cb(msg, name=name):
            now = node.get_clock().now().nanoseconds
            st = msg.header.stamp
            ages[name].append((now - (st.sec * 10**9 + st.nanosec)) / 1e6)
        node.create_subscription(get_message(typ), name, cb, qos_profile_sensor_data)
    end = time.time() + a.secs
    nxt = time.time() + 1
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= nxt:
            nxt += 1
            parts = []
            for name, v in ages.items():
                short = name.rsplit('/', 1)[-1]
                parts.append(f'{short} n={len(v)} med={statistics.median(v):.0f} max={max(v):.0f}'
                             if v else f'{short} n=0')
                v.clear()
            print(time.strftime('%H:%M:%S'), ' | '.join(parts), flush=True)


if __name__ == '__main__':
    sys.exit(main())
