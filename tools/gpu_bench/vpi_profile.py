#!/usr/bin/env python3
"""Where does the VPI path spend CPU? Times variants of the matcher call on real rectified pairs.

    PYTHONPATH=/tmp/gpubench python3 vpi_profile.py /tmp/gpubench/pairs.npz
"""
import sys
import time

import numpy as np
import vpi

sys.path.insert(0, '/tmp/gpubench/overwall_nav_pkg')
from overwall_nav.stereo import StereoRig, extrinsics_from_tf   # noqa: E402

data = dict(np.load(sys.argv[1]))
r, t = extrinsics_from_tf(data['tf_t'], data['tf_q'])
rig = StereoRig(data['k_left'], data['k_right'], tuple(int(v) for v in data['size']), r, t)
r0, r1 = rig.rows
pairs = []
for i in range(len(data['left'])):
    lr, rr = rig.rectify(data['left'][i], data['right'][i])
    pairs.append((np.ascontiguousarray(lr[r0:r1]), np.ascontiguousarray(rr[r0:r1])))
ARGS = dict(window=5, maxdisp=96, mindisp=0, p1=3, p2=48, confthreshold=32767, quality=6, uniqueness=-1.0)
stream = vpi.Stream()


def y16_per_call(lf, rf):
    with stream, vpi.Backend.CUDA:
        lv = vpi.asimage(lf).convert(vpi.Format.Y16_ER)
        rv = vpi.asimage(rf).convert(vpi.Format.Y16_ER)
        d = vpi.stereodisp(lv, rv, **ARGS)
    stream.sync()
    return np.array(d.cpu(), copy=True)


def u8_direct(lf, rf):
    with stream, vpi.Backend.CUDA:
        d = vpi.stereodisp(vpi.asimage(lf), vpi.asimage(rf), **ARGS)
    stream.sync()
    return np.array(d.cpu(), copy=True)


h, w = pairs[0][0].shape
L, R = vpi.Image((w, h), vpi.Format.U8), vpi.Image((w, h), vpi.Format.U8)


def u8_prealloc(lf, rf):
    with L.wlock_cpu() as a:
        a[:] = lf
    with R.wlock_cpu() as a:
        a[:] = rf
    with stream, vpi.Backend.CUDA:
        d = vpi.stereodisp(L, R, **ARGS)
    stream.sync()
    with d.rlock_cpu() as a:
        return a.copy()


def gpu_only(lf, rf):          # same inputs every call: matcher + sync only
    with stream, vpi.Backend.CUDA:
        d = vpi.stereodisp(L, R, **ARGS)
    stream.sync()
    return d


for name, fn in (('y16_per_call', y16_per_call), ('u8_direct', u8_direct), ('u8_prealloc', u8_prealloc),
                 ('gpu_only', gpu_only)):
    try:
        for lf, rf in pairs[:3]:
            fn(lf, rf)
        wall, cpu = [], []
        for lf, rf in pairs:
            c0, w0 = time.process_time(), time.perf_counter()
            fn(lf, rf)
            wall.append((time.perf_counter() - w0) * 1e3)
            cpu.append((time.process_time() - c0) * 1e3)
        print(f'{name:13s} wall median {np.median(wall):5.1f} ms p90 {np.percentile(wall, 90):5.1f} | '
              f'CPU {np.median(cpu):5.1f} ms/frame', flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f'{name:13s} failed: {type(exc).__name__}: {exc}', flush=True)
