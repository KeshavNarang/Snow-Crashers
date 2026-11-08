#!/usr/bin/env python3
"""Benchmark stereo_depth's matcher: CPU SGBM (what flies) against VPI on the Jetson's GPU / OFA.

Every method runs through the same ``StereoRig`` (same rectification, texture/saturation filters
and 3D points); only the disparity matcher changes. Three parts:

  accuracy   exact ray-traced pairs (team_ws/.../test/synthetic.py): wall distance at 0.5-2.5 m,
             floor pitch/height, against ground truth.
  real       captured live pairs (capture_pairs.py): per-frame ms (rectify / match / post),
             CPU-seconds per frame, points, floor fits, disparity agreement with SGBM.
  soak       --soak S: loop one method flat out for S seconds -> pairs/s and CPU %, to run while
             watching cuVSLAM's rate and tegrastats.

    python3 stereo_bench.py --pairs /tmp/pairs.npz --json /tmp/bench.json
    python3 stereo_bench.py --pairs /tmp/pairs.npz --soak 20 --methods vpi-cuda
Needs overwall_nav on PYTHONPATH (and the vpi module for the VPI methods).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE.parents[1] / 'team_ws/src/overwall_nav', HERE / 'overwall_nav_pkg'):
    sys.path.insert(0, str(p))
    sys.path.insert(0, str(p / 'test'))

from overwall_nav.perception import analyze, bin_points                           # noqa: E402
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, fit_floor, level   # noqa: E402

PITCH = 27.0
CORRIDOR = 0.4
# name -> StereoRig arguments. 'sgbm' is exactly what flies (config/overwall.yaml defaults).
METHODS = {
    'sgbm':          dict(matcher='sgbm', scale=0.5, num_disparities=96, step=2),
    'vpi-cuda':      dict(matcher='vpi-cuda', scale=0.5, num_disparities=96, step=2),
    'vpi-cuda-full': dict(matcher='vpi-cuda', scale=1.0, num_disparities=192, step=4),
    'vpi-ofa':       dict(matcher='vpi-ofa', scale=0.5, num_disparities=128, step=2),
    'sgbm-full':     dict(matcher='sgbm', scale=1.0, num_disparities=192, step=4),
}


def stats(xs):
    xs = np.asarray(xs, dtype=float)
    if xs.size == 0:
        return None
    return {'median': round(float(np.median(xs)), 2), 'p90': round(float(np.percentile(xs, 90)), 2),
            'mean': round(float(xs.mean()), 2)}


def make_rig(name, k_left, k_right, size, r, t, d_left=None, d_right=None):
    return StereoRig(k_left, k_right, size, r, t, d_left=d_left, d_right=d_right, **METHODS[name])


def disparity(rig, left, right):
    lr, rr = rig.rectify(left, right)
    r0, r1 = rig.rows
    d = rig.matcher.compute(lr[r0:r1], rr[r0:r1]).astype(np.float32) / 16.0
    d[d >= rig.num_disparities - rig.saturation_px] = 0.0
    if rig.scale != 0.5:                        # compare on the half-resolution grid
        d = d[::2, ::2] * (0.5 / rig.scale)
    return d


# -- accuracy on exact synthetic pairs -------------------------------------------------------------
def accuracy(names):
    import synthetic
    r, t = extrinsics_from_tf(synthetic.T_LR, synthetic.Q_LR)
    out = {}
    walls = [(d, synthetic.pair(cam_height=0.85, wall_distance=d, wall_top=1.0)) for d in (0.5, 0.8, 1.5, 2.5)]
    floors = [((p, h), synthetic.pair(pitch_deg=p, cam_height=h)) for p, h in ((27.0, 0.14), (27.0, 0.85), (20.0, 0.5))]
    tops = [(h, synthetic.pair(cam_height=h, wall_distance=0.5, wall_top=1.0)) for h in (0.85, 1.2, 1.3, 1.45)]
    for name in names:
        rig = make_rig(name, synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, r, t)
        top = rig.top_elevation_deg(PITCH)
        res = {'wall': [], 'floor': [], 'top': []}
        for d, (left, right) in walls:
            pts, valid = rig.points(left, right)
            a = analyze(bin_points(level(pts, PITCH), CORRIDOR), 0.85, top)
            res['wall'].append({'true_m': d, 'front_m': a['front'],
                                'err_m': None if a['front'] is None else round(a['front'] - d, 3),
                                'valid': round(valid, 3)})
        for (p, h), (left, right) in floors:
            pts, valid = rig.points(left, right)
            f = fit_floor(pts)
            res['floor'].append({'true': [p, h], 'fit': None if f is None else [f['pitch_deg'], f['height_m']],
                                 'err_deg': None if f is None else round(f['pitch_deg'] - p, 2),
                                 'err_m': None if f is None else round(f['height_m'] - h, 3)})
        for h, (left, right) in tops:          # wall top (1.0 m) hidden at 0.85 m, visible above ~1.15
            pts, _ = rig.points(left, right)
            a = analyze(bin_points(level(pts, PITCH), CORRIDOR), h, top, wall_distance=0.5)
            res['top'].append({'cam_h': h, 'top_visible': a['top_visible'],
                               'top_m': None if a['top_u'] is None else round(h + a['top_u'], 3)})
        we = [abs(w['err_m']) for w in res['wall'] if w['err_m'] is not None]
        res['wall_max_abs_err_m'] = round(max(we), 3) if len(we) == len(walls) else 'MISSED A WALL'
        out[name] = res
        print(f'  {name:14s} wall errors {[w["err_m"] for w in res["wall"]]} m | floor '
              f'{[(f["err_deg"], f["err_m"]) for f in res["floor"]]} (deg, m) | top visible '
              f'{[(t["cam_h"], t["top_visible"], t["top_m"]) for t in res["top"]]}', flush=True)
    return out


# -- real captured frames --------------------------------------------------------------------------
def real(names, data, warmup=3):
    r, t = extrinsics_from_tf(data['tf_t'], data['tf_q'])
    cal = dict(k_left=data['k_left'], k_right=data['k_right'], size=tuple(int(v) for v in data['size']), r=r, t=t,
               d_left=data['d_left'] if len(data['d_left']) else None,
               d_right=data['d_right'] if len(data['d_right']) else None)
    lefts, rights = data['left'], data['right']
    n = len(lefts)
    rigs = {name: make_rig(name, **cal) for name in names}
    ref = None
    if 'sgbm' in rigs:
        ref = [disparity(rigs['sgbm'], lefts[i], rights[i]) for i in range(n)]
    out = {}
    for name, rig in rigs.items():
        for i in range(min(warmup, n)):                       # CUDA context, allocations, caches
            rig.points(lefts[i], rights[i])
        ms = {'total': [], 'rectify': [], 'match': [], 'post': [], 'cpu': []}
        npts, floors, fronts, valid = [], [], [], []
        for i in range(n):
            tm = {}
            c0, w0 = time.process_time(), time.perf_counter()
            pts, v = rig.points(lefts[i], rights[i], timings=tm)
            w1, c1 = time.perf_counter(), time.process_time()
            ms['total'].append((w1 - w0) * 1e3)
            ms['cpu'].append((c1 - c0) * 1e3)
            for k in ('rectify', 'match', 'post'):
                ms[k].append(tm[k] * 1e3)
            npts.append(len(pts))
            valid.append(v)
            f = fit_floor(pts)
            floors.append(f)
            a = analyze(bin_points(level(pts, PITCH), CORRIDOR), f['height_m'] if f else None,
                        rig.top_elevation_deg(PITCH))
            fronts.append(a['front'])
        res = {k + '_ms': stats(v) for k, v in ms.items()}
        res['points'] = stats(npts)
        res['matched'] = stats(valid)
        good = [f for f in floors if f]
        res['floor_fits'] = f'{len(good)}/{n}'
        res['floor_pitch_deg'] = stats([f['pitch_deg'] for f in good])
        res['floor_height_m'] = stats([f['height_m'] for f in good])
        res['front_m'] = {'clear': sum(f is None for f in fronts), 'seen': stats([f for f in fronts if f is not None])}
        if ref is not None and name != 'sgbm':
            agree, both, within1 = [], [], []
            for i in range(n):
                d = disparity(rig, lefts[i], rights[i])
                h, w = min(d.shape[0], ref[i].shape[0]), min(d.shape[1], ref[i].shape[1])
                a, b = d[:h, :w], ref[i][:h, :w]
                m = (a > 0.5) & (b > 0.5)
                both.append(m.mean())
                if m.any():
                    diff = np.abs(a[m] - b[m])
                    agree.append(float(np.median(diff)))
                    within1.append(float((diff <= 1.0).mean()))
            res['vs_sgbm'] = {'both_valid': stats(both), 'median_abs_diff_px': stats(agree),
                              'within_1px': stats(within1)}
        out[name] = res
        print(f'  {name:14s} total {res["total_ms"]["median"]:6.1f} ms (p90 {res["total_ms"]["p90"]:.1f}) = rectify '
              f'{res["rectify_ms"]["median"]:.1f} + match {res["match_ms"]["median"]:.1f} + post '
              f'{res["post_ms"]["median"]:.1f} | CPU {res["cpu_ms"]["median"]:.1f} ms/frame | points '
              f'{res["points"]["median"]:.0f} | floor {res["floor_fits"]}, pitch {res["floor_pitch_deg"]} | '
              f'front clear {res["front_m"]["clear"]}/{n}'
              + (f' | vs SGBM within 1 px {res["vs_sgbm"]["within_1px"]}' if 'vs_sgbm' in res else ''), flush=True)
    return out


def soak(name, data, seconds):
    r, t = extrinsics_from_tf(data['tf_t'], data['tf_q'])
    rig = make_rig(name, data['k_left'], data['k_right'], tuple(int(v) for v in data['size']), r, t)
    lefts, rights = data['left'], data['right']
    rig.points(lefts[0], rights[0])
    n, c0, w0 = 0, time.process_time(), time.perf_counter()
    while time.perf_counter() - w0 < seconds:
        i = n % len(lefts)
        pts, _ = rig.points(lefts[i], rights[i])
        fit_floor(pts)
        n += 1
    wall, cpu = time.perf_counter() - w0, time.process_time() - c0
    res = {'method': name, 'seconds': round(wall, 1), 'frames': n, 'fps': round(n / wall, 1),
           'cpu_percent': round(100 * cpu / wall), 'cpu_ms_per_frame': round(1e3 * cpu / n, 1)}
    print(json.dumps(res), flush=True)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pairs', help='.npz from capture_pairs.py')
    ap.add_argument('--methods', default='sgbm,vpi-cuda,vpi-cuda-full,vpi-ofa')
    ap.add_argument('--skip-accuracy', action='store_true')
    ap.add_argument('--soak', type=float, default=0.0)
    ap.add_argument('--json')
    a = ap.parse_args()
    names = [m for m in a.methods.split(',') if m]
    ok = []
    for m in names:          # drop methods this machine cannot build (no vpi module, no OFA engine)
        try:
            StereoRig(np.eye(3) * [[600], [600], [1]], np.eye(3) * [[600], [600], [1]], (1024, 640),
                      np.eye(3), np.array([[-0.075], [0], [0]]), **METHODS[m]).matcher.compute(
                np.zeros((288, 512), np.uint8) + 1, np.zeros((288, 512), np.uint8) + 1)
            ok.append(m)
        except Exception as exc:  # noqa: BLE001
            print(f'  {m}: unavailable here ({type(exc).__name__}: {exc})', flush=True)
    names = ok
    data = dict(np.load(a.pairs)) if a.pairs else None
    if a.soak:
        res = {'soak': [soak(m, data, a.soak) for m in names]}
    else:
        res = {'methods': {m: METHODS[m] for m in names}}
        if not a.skip_accuracy:
            print('accuracy (synthetic, exact ground truth):', flush=True)
            res['accuracy'] = accuracy(names)
        if data is not None:
            print(f'real frames ({len(data["left"])} pairs {data["left"].shape[2]}x{data["left"].shape[1]}):', flush=True)
            res['real'] = real(names, data)
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1, default=str))


if __name__ == '__main__':
    main()
