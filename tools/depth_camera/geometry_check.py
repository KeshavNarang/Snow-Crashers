#!/usr/bin/env python3
"""geometry_check: is the OAK's on-device depth in the frame its camera_info claims, and how accurate is it?

Read-only. Inside the skybeta-stack container (ROS + OpenCV), with a depth configuration running:

    python3 geometry_check.py [--n 5] [--out /root/team_ws/depth_state/geometry.json]

For n instants it takes the depth image and the left/right mono images with the SAME header stamp, rectifies the
pair with the cameras' own CameraInfo (K, D, R, P), matches it on the host with OpenCV SGBM (depth in the rectified
left and right frames), moves that host depth into each frame the device depth could be in (frames.py), and asks
which one the device depth matches (median |error| <= 3 %). Pass (acceptance B2): the best frame is the one the
depth's camera_info claims, and the best extra horizontal shift is 0 px.
"""
import argparse
import json
import os
import time

import numpy as np

import depth_analysis as da
import frames
import geometry as g

CLAIMED = 'claimed: raw left (its camera_info)'


def match_triplets(lefts, rights, depths, tol_s=0.003):
    """[(stamp, left, right, depth)] for depth frames with a left and a right image from the same instant.
    Each input is [(stamp, payload)]; no left/right image is used twice."""
    used_l, used_r, out = set(), set(), []

    def nearest(items, used, t):
        best = None
        for i, (s, _) in enumerate(items):
            if i in used or abs(s - t) > tol_s:
                continue
            if best is None or abs(s - t) < abs(items[best][0] - t):
                best = i
        return best

    for t, d in sorted(depths, key=lambda x: x[0]):
        il, ir = nearest(lefts, used_l, t), nearest(rights, used_r, t)
        if il is None or ir is None:
            continue
        used_l.add(il)
        used_r.add(ir)
        out.append((t, lefts[il][1], rights[ir][1], d))
    return out


def _info(m):
    return {'width': m.width, 'height': m.height, 'k': list(m.k), 'd': list(m.d), 'r': list(m.r), 'p': list(m.p),
            'frame_id': m.header.frame_id, 'distortion_model': m.distortion_model}


def _mono(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * (3 if m.encoding in ('bgr8', 'rgb8') else 1)]
    return a.reshape(m.height, m.width, -1)[..., 0].copy() if m.encoding in ('bgr8', 'rgb8') else a.reshape(m.height, m.width).copy()


def collect(n=5, timeout_s=40.0, left='/oak/left/image_mono', right='/oak/right/image_mono', depth='/oak/stereo/image_raw'):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = Node('depth_geometry_check')
    buf = {'l': [], 'r': [], 'd': []}
    info = {}

    def keep(key, msg):
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        buf[key].append((s, msg))
        if key in ('l', 'r') and len(buf[key]) > 60:
            del buf[key][0]

    for key, topic in (('l', left), ('r', right), ('d', depth)):
        node.create_subscription(Image, topic, lambda m, k=key: keep(k, m), qos_profile_sensor_data)
    for key, topic in (('l', '/oak/left/camera_info'), ('r', '/oak/right/camera_info'), ('d', '/oak/stereo/camera_info')):
        node.create_subscription(CameraInfo, topic, lambda m, k=key: info.setdefault(k, _info(m)), qos_profile_sensor_data)
    t0, trip = time.time(), []
    try:
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            trip = match_triplets(buf['l'], buf['r'], buf['d'])
            if len(trip) >= n and len(info) == 3:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return trip[:n], info, {k: len(v) for k, v in buf.items()}


def analyse(trip, info, num_disp=128, block=7, save=None):
    import cv2
    il, ir, idv = info['l'], info['r'], info['d']
    host_size = (il['width'], il['height'])
    dev_size = (idv['width'], idv['height'])
    maps = {}
    for key, inf in (('l', il), ('r', ir)):
        k = np.array(inf['k'], float).reshape(3, 3)
        maps[key] = cv2.initUndistortRectifyMap(k, np.array(inf['d'], float), np.array(inf['r'], float).reshape(3, 3),
                                                np.array(inf['p'], float).reshape(3, 4), host_size, cv2.CV_32FC1)
    base = frames.baseline(il, ir)
    fxp = il['p'][0]
    rows = []
    for idx, (t, lm, rm, dm) in enumerate(trip):
        lr = cv2.remap(_mono(lm), *maps['l'], cv2.INTER_LINEAR)
        rr = cv2.remap(_mono(rm), *maps['r'], cv2.INTER_LINEAR)
        dl, dr = g.host_depth(lr, rr, fxp, base, num_disp=num_disp, block=block)
        cands = frames.build_candidates(dl, dr, il, ir, idv, host_size, dev_size)
        dev = da.decode_depth(dm.data, dm.height, dm.width, dm.encoding, step=dm.step, is_bigendian=dm.is_bigendian)
        ident = g.identify_frame(dev, cands)
        shift, score = da.best_shift(dev, cands[CLAIMED], max_shift=12)
        rows.append({'stamp': t, 'best': ident['best'], 'shift_vs_claimed_px': shift, 'shift_score': score,
                     'scores': ident['scores'], 'device_valid': da.valid_fraction(dev),
                     'host_valid_claimed': da.valid_fraction(cands[CLAIMED]), 'encoding': dm.encoding})
        if save and idx == 0:
            np.savez_compressed(save, device=dev, rect_left=lr, rect_right=rr, host_left=dl, host_right=dr,
                                **{k.split(':')[0].replace(' ', '_'): v for k, v in cands.items()})
    votes = {}
    for r in rows:
        votes[r['best']] = votes.get(r['best'], 0) + 1
    med = {name: float(np.median([r['scores'][name]['median_rel'] for r in rows if r['scores'][name]['median_rel'] is not None]))
           for name in (rows[0]['scores'] if rows else {})
           if any(r['scores'][name]['median_rel'] is not None for r in rows)}
    shifts = [r['shift_vs_claimed_px'] for r in rows if r['shift_vs_claimed_px'] is not None]
    verdict = {'samples': len(rows), 'votes': votes, 'median_rel_by_frame': med,
               'shift_vs_claimed_px': int(np.median(shifts)) if shifts else None,
               'baseline_m': base, 'depth_frame_id': idv['frame_id'], 'depth_size': dev_size, 'host_size': host_size,
               'r_left': il['r'], 'r_right': ir['r'],
               'device_valid': float(np.median([r['device_valid'] for r in rows])) if rows else None,
               'host_valid_claimed': float(np.median([r['host_valid_claimed'] for r in rows])) if rows else None}
    best = max(votes, key=votes.get) if votes else None
    verdict['best_frame'] = best
    verdict['b2_pass'] = bool(best == CLAIMED and verdict['shift_vs_claimed_px'] == 0
                              and med.get(CLAIMED, 1.0) <= 0.03)
    return verdict, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--n', type=int, default=5)
    ap.add_argument('--out', default='/root/team_ws/depth_state/geometry.json')
    a = ap.parse_args()
    trip, info, counts = collect(a.n)
    if len(trip) == 0 or len(info) < 3:
        print(json.dumps({'error': 'no synchronized left/right/depth set', 'received': counts, 'infos': sorted(info)}))
        return 1
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    verdict, rows = analyse(trip, info, save=a.out.replace('.json', '_sample.npz'))
    with open(a.out, 'w') as f:
        json.dump({'verdict': verdict, 'rows': rows}, f, indent=1, default=str)
    print(json.dumps(verdict, indent=1, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
