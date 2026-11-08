#!/usr/bin/env python3
"""Listen to a stereo_depth output topic for S seconds -> one JSON line: Hz, proc_ms, age_ms, points.

    python3 topic_stats.py /overwall/front_test 20
"""
import json
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    topic, seconds = sys.argv[1], float(sys.argv[2])
    rclpy.init(args=[])
    node = Node('gpu_bench_topic_stats')
    got = []
    node.create_subscription(String, topic, lambda m: got.append((time.monotonic(), json.loads(m.data))), 10)
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    def med(key):
        xs = [g[1][key] for g in got if g[1].get(key) is not None]
        return round(statistics.median(xs), 1) if xs else None
    def p90(key):
        xs = sorted(g[1][key] for g in got if g[1].get(key) is not None)
        return round(xs[int(0.9 * (len(xs) - 1))], 1) if xs else None
    hz = (len(got) - 1) / (got[-1][0] - got[0][0]) if len(got) > 1 else 0.0
    floors = [g[1].get('floor') for g in got]
    print(json.dumps({'topic': topic, 'msgs': len(got), 'hz': round(hz, 2), 'proc_ms': med('proc_ms'),
                      'proc_ms_p90': p90('proc_ms'), 'age_ms': med('age_ms'), 'age_ms_p90': p90('age_ms'),
                      'points': med('n'), 'floor_fits': f'{sum(1 for f in floors if f)}/{len(floors)}'}), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
