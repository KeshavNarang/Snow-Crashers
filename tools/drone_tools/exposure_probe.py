#!/usr/bin/env python3
"""exposure_probe: record how bright/sharp/textured the camera's frames are over time, then analyse it.

Purpose: find out how the camera copes when the light changes (hall -> dark tunnel -> hall), WITHOUT
assuming anything about auto-exposure: it measures the frames themselves. Read-only: it subscribes,
publishes nothing and changes no parameter.

Record (inside the skybeta-stack container, ROS sourced). While it runs, change the light: cover both
lenses with a cloth for several seconds, then uncover them (or carry the drone into the dark and back):
    python3 /root/team_ws/tools/drone_tools/exposure_probe.py record --secs 45 --out /tmp/exp.jsonl

Analyse (anywhere; no ROS needed):
    python3 exposure_probe.py analyse /tmp/exp.jsonl [--side left]

Each frame is reduced to a few numbers (see exposure_analysis.py) at --hz frames per second per eye, using
the mono8 topics by default (same pixels as the driver's bgr8 output; the 'grey' flag in every record
confirms the three channels are really identical when bgr8 is used).
"""
import argparse
import json
import sys
import time

import numpy as np

import exposure_analysis as ea

try:                                    # ROS exists on the drone only; the decoding and throttling are testable anywhere
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
except ImportError:
    rclpy, Node, Image = None, object, None


def image_to_gray(m):
    """(2-D uint8 grey array, channels_were_equal) from a sensor_msgs/Image-like object. Handles row padding
    (`step` larger than the row). bgr8 is reduced to the mean of its channels and flagged if they differ."""
    channels = {'mono8': 1, '8UC1': 1, 'bgr8': 3, 'rgb8': 3}.get(m.encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding {m.encoding!r}')
    h, w = m.height, m.width
    step = m.step or w * channels
    if len(m.data) < h * step or step < w * channels:
        raise ValueError(f'image data too short: {len(m.data)} bytes for {h} rows of {step}')
    a = np.frombuffer(bytes(m.data), np.uint8)[:h * step].reshape(h, step)[:, :w * channels]
    if channels == 1:
        return a.reshape(h, w).copy(), True
    a = a.reshape(h, w, 3)
    equal = bool(np.array_equal(a[..., 0], a[..., 1]) and np.array_equal(a[..., 1], a[..., 2]))
    return a.mean(axis=2).astype(np.uint8), equal


class Throttle:
    """At most `hz` frames per second for each key, on a fixed grid (a rejected frame does not move the clock)."""

    def __init__(self, hz):
        if hz <= 0:
            raise ValueError('rate must be positive')
        self.period, self.next = 1.0 / hz, {}

    def allow(self, key, now):
        due = self.next.get(key)
        if due is not None and now < due - 1e-9:
            return False
        self.next[key] = (due + self.period) if due is not None and now - due < self.period else now + self.period
        return True


def make_record(side, m, arrival):
    grey, equal = image_to_gray(m)
    stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    return {'t': stamp, 'arrival': arrival, 'age_ms': (arrival - stamp) * 1e3, 'side': side, 'grey': equal,
            **ea.frame_metrics(grey)}


class Recorder(Node):
    def __init__(self, sides, suffix, hz):
        super().__init__('exposure_probe')
        self.records, self.throttle, self.errors = [], Throttle(hz), 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        for side in sides:
            self.create_subscription(Image, f'/oak/{side}/{suffix}', lambda m, s=side: self.on(s, m), qos)

    def on(self, side, m):
        now = time.time()
        if not self.throttle.allow(side, now):
            return
        try:
            self.records.append(make_record(side, m, now))
        except ValueError:
            self.errors += 1


def record(a):
    if rclpy is None:
        raise SystemExit('ROS 2 not found: run this inside the skybeta-stack container (source /opt/ros/humble/setup.bash)')
    rclpy.init()
    node = Recorder(a.sides.split(','), a.suffix, a.hz)
    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    recs = node.records
    with open(a.out, 'w') as f:
        for r in recs:
            f.write(json.dumps(r) + '\n')
    per = {s: sum(1 for r in recs if r['side'] == s) for s in a.sides.split(',')}
    print(f'recorded {len(recs)} frames {per} in {time.time() - t0:.0f} s -> {a.out}; unusable frames skipped: {node.errors}')
    nongrey = sum(1 for r in recs if not r['grey'])
    if nongrey:
        print(f'WARNING: {nongrey} frames had different B/G/R channels, so they are not plain grey')
    node.destroy_node()
    rclpy.shutdown()


def analyse(a):
    frames, skipped = ea.load_frames(a.file)
    if skipped:
        print(f'(skipped {skipped} unreadable lines)')
    sides = [a.side] if a.side else sorted({f.get('side') for f in frames if f.get('side')})
    for side in sides:
        print(f'=== {side} ===')
        try:
            print(ea.format_report(ea.summarise(frames, side)))
        except ValueError as e:
            print(f'cannot analyse: {e}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('--secs', type=float, default=45.0)
    r.add_argument('--hz', type=float, default=5.0)
    r.add_argument('--out', required=True)
    r.add_argument('--sides', default='left,right')
    r.add_argument('--suffix', default='image_mono', help='topic: /oak/<side>/<suffix>')
    r.set_defaults(fn=record)
    n = sub.add_parser('analyse')
    n.add_argument('file')
    n.add_argument('--side')
    n.set_defaults(fn=analyse)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
