#!/usr/bin/env python3
"""Save N synced stereo pairs + calibration from the live stack to an .npz (subscribes only).

Run inside the skybeta container, stack up:
    python3 capture_pairs.py --out /tmp/pairs.npz --count 40 --every 3

Saves left, right (N x H x W uint8, /oak/*/image_mono), k_left, k_right, d_left, d_right, size,
tf_t, tf_q (pose of the right optical frame in the left one, as stereo_depth looks it up), stamps.
"""
import argparse
import time

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/pairs.npz')
    ap.add_argument('--count', type=int, default=40)
    ap.add_argument('--every', type=int, default=3, help='keep one synced pair in this many')
    ap.add_argument('--timeout', type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node('gpu_bench_capture')
    bridge = CvBridge()
    info, frames = {}, []
    seen = [0]
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    node.create_subscription(CameraInfo, '/oak/left/camera_info', lambda m: info.setdefault('left', m), 1)
    node.create_subscription(CameraInfo, '/oak/right/camera_info', lambda m: info.setdefault('right', m), 1)
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
    left = message_filters.Subscriber(node, Image, '/oak/left/image_mono', qos_profile=qos)
    right = message_filters.Subscriber(node, Image, '/oak/right/image_mono', qos_profile=qos)
    sync = message_filters.ApproximateTimeSynchronizer([left, right], queue_size=4, slop=0.01)

    def on_pair(lm, rm):
        seen[0] += 1
        if seen[0] % a.every == 0 and len(frames) < a.count:
            frames.append((bridge.imgmsg_to_cv2(lm, 'mono8').copy(), bridge.imgmsg_to_cv2(rm, 'mono8').copy(),
                           Time.from_msg(lm.header.stamp).nanoseconds * 1e-9))
    sync.registerCallback(on_pair)

    start = time.monotonic()
    while (len(frames) < a.count or len(info) < 2) and time.monotonic() - start < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not frames or len(info) < 2:
        raise SystemExit(f'got {len(frames)} pairs, camera_info {sorted(info)}: is the stack running?')
    li, ri = info['left'], info['right']
    tf = tf_buffer.lookup_transform(li.header.frame_id, ri.header.frame_id, Time())
    tr, q = tf.transform.translation, tf.transform.rotation
    np.savez_compressed(
        a.out, left=np.stack([f[0] for f in frames]), right=np.stack([f[1] for f in frames]),
        stamps=np.array([f[2] for f in frames]), k_left=np.array(li.k), k_right=np.array(ri.k),
        d_left=np.array(li.d), d_right=np.array(ri.d), size=np.array([li.width, li.height]),
        tf_t=np.array([tr.x, tr.y, tr.z]), tf_q=np.array([q.x, q.y, q.z, q.w]))
    span = frames[-1][2] - frames[0][2]
    print(f'saved {len(frames)} pairs over {span:.1f} s ({seen[0]} synced pairs seen) -> {a.out}; '
          f'{li.header.frame_id} -> {ri.header.frame_id} t={[round(v, 4) for v in (tr.x, tr.y, tr.z)]}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
