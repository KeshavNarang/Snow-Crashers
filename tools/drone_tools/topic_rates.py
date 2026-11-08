#!/usr/bin/env python3
"""topic_rates: how fast, how steady and how old are the sensor topics? (low overhead)

`ros2 topic hz` deserializes every message in Python, which for a 2 MB image costs enough CPU to
change the very rate being measured. This subscribes with raw=True (the serialized bytes are never
unpacked) and reads only the 8-byte header time stamp out of the CDR header.

Run inside the container (ROS sourced):
    python3 /root/team_ws/tools/topic_rates.py                 default sensor set, 10 s
    python3 /root/team_ws/tools/topic_rates.py --secs 20 /oak/left/image_mono /oak/imu/data
    python3 /root/team_ws/tools/topic_rates.py --json

Per topic: rate, median and worst gap between messages, how many gaps were over 1.5x the median
(dropped or late frames), and how old each message was on arrival (now - header.stamp: the
pipeline latency up to this process). With a left/right pair it also reports the stamp offset
between the eyes (should be ~0: a whole frame period means stereo pairs are mismatched).
Publishes nothing.
"""
import argparse
import json
import struct
import time

import numpy as np

try:                                    # ROS exists on the drone only; the maths below is testable anywhere
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    rclpy, Node = None, object

DEFAULT = ['/oak/left/image_mono', '/oak/right/image_mono', '/oak/left/image_raw', '/oak/imu/data',
           '/visual_slam/tracking/odometry']


def cdr_stamp(raw):
    """header.stamp in seconds from a serialized message, or None if it is too short.
    CDR: a 4-byte encapsulation header (byte 1 is 1 for little endian), then Header.stamp as
    int32 sec and uint32 nanosec."""
    if len(raw) < 12:
        return None
    sec, nsec = struct.unpack('<iI' if raw[1] == 1 else '>iI', bytes(raw[4:12]))
    return sec + nsec * 1e-9


def resolve_types(node, topics, timeout=5.0, poll=0.25):
    """{topic: type name} for the wanted topics. DDS discovery is asynchronous: right after a node
    starts the graph is empty, so poll until every topic is found or the timeout passes."""
    deadline = time.time() + timeout
    while True:
        types = {name: kinds[0] for name, kinds in node.get_topic_names_and_types() if name in topics and kinds}
        if len(types) == len(set(topics)) or time.time() >= deadline:
            return types
        time.sleep(poll)
        if hasattr(node, 'context') and rclpy is not None:
            rclpy.spin_once(node, timeout_sec=0.0)


class Probe(Node):
    def __init__(self, topics):
        super().__init__('topic_rates_probe')
        self.rec = {}
        types = resolve_types(self, topics)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        for t in topics:
            if t not in types:
                self.rec[t] = None
                continue
            self.rec[t] = {'arr': [], 'stamp': []}
            self.create_subscription(get_message(types[t]), t, lambda b, t=t: self.on(t, b), qos, raw=True)

    def on(self, topic, raw):
        now = time.time()
        stamp = cdr_stamp(raw)
        if stamp is None:
            return
        r = self.rec[topic]
        r['arr'].append(now)
        r['stamp'].append(stamp)


def summarise(rec, secs):
    out = {}
    for t, r in rec.items():
        if r is None:
            out[t] = {'error': 'topic not found'}
            continue
        a, s = np.array(r['arr']), np.array(r['stamp'])
        if len(a) < 3:
            out[t] = {'error': f'only {len(a)} messages'}
            continue
        gap = np.diff(a)
        med = float(np.median(gap))
        age = a - s
        out[t] = {'hz': round(len(a) / secs, 1), 'n': len(a), 'median_gap_ms': round(med * 1e3, 1),
                  'worst_gap_ms': round(float(gap.max()) * 1e3), 'late_gaps': int((gap > 1.5 * med).sum()),
                  'age_median_ms': round(float(np.median(age)) * 1e3), 'age_p95_ms': round(float(np.percentile(age, 95)) * 1e3),
                  'age_max_ms': round(float(age.max()) * 1e3)}
    for left, right in (('/oak/left/image_mono', '/oak/right/image_mono'), ('/oak/left/image_raw', '/oak/right/image_raw')):
        if rec.get(left) and rec.get(right) and len(rec[left]['stamp']) > 3 and len(rec[right]['stamp']) > 3:
            r_s = np.array(rec[right]['stamp'])
            off = [float(np.min(np.abs(r_s - x))) for x in rec[left]['stamp']]
            out[f'{left} vs {right}'] = {'stamp_offset_median_ms': round(float(np.median(off)) * 1e3, 1),
                                         'stamp_offset_max_ms': round(max(off) * 1e3, 1)}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('topics', nargs='*', default=DEFAULT)
    ap.add_argument('--secs', type=float, default=10.0)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    if rclpy is None:
        raise SystemExit('ROS 2 not found: run this inside the skybeta-stack container (source /opt/ros/humble/setup.bash)')
    rclpy.init()
    node = Probe(a.topics)
    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    res = summarise(node.rec, time.time() - t0)
    if a.json:
        print(json.dumps(res, indent=1))
    else:
        for t, v in res.items():
            if 'error' in v:
                print(f'{t:<36} {v["error"]}')
            elif 'hz' in v:
                print(f'{t:<36} {v["hz"]:6.1f} Hz | gap median {v["median_gap_ms"]:6.1f} ms worst {v["worst_gap_ms"]:4d} ms '
                      f'late {v["late_gaps"]:3d} | age median {v["age_median_ms"]:4d} p95 {v["age_p95_ms"]:4d} max {v["age_max_ms"]:4d} ms')
            else:
                print(f'{t:<60} stamp offset median {v["stamp_offset_median_ms"]} ms, max {v["stamp_offset_max_ms"]} ms')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
