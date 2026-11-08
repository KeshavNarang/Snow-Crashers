# GPU stereo on skybeta-03 (2026-11-07, branch agent/gpu-stereo)

Question: the Jetson's GPU idles (tegrastats GR3D 0-12 %) while all 6 CPU cores run 60-80 %;
stereo_depth's CPU SGBM is the biggest CPU user. Can the depth matcher move to the GPU?

## What is on the drone
- Orin Nano Super, L4T R36.5 (JetPack 6.2): CUDA 12.6, TensorRT 10.3, cuDNN 9, **VPI 3.2.4**.
- Host: `python3.10-vpi3` (the `vpi` Python module). Container (skybeta:0.3.3): `libnvvpi.so.3.2.4`
  in /opt/nvidia/vpi3 but no Python module; OpenCV 4.5.4 without CUDA. Runtime is `nvidia`.
- The host's `vpi.cpython-310-aarch64-linux-gnu.so` imports fine inside the container (same
  Python 3.10, same VPI version) via PYTHONPATH. Correction to the depth-bottleneck log: VPI's
  library *is* in the container, only the binding is missing.
- VPI backends available: CUDA and **OFA** (Orin's optical-flow engine) both run stereo here.

## Code (branch agent/gpu-stereo, not deployed)
- `overwall_nav/vpi_stereo.py`: `VpiMatcher`, a drop-in for cv2.StereoSGBM (same 1/16 px output).
- `StereoRig(..., matcher='sgbm'|'vpi-cuda'|'vpi-ofa')`, default unchanged (sgbm);
  `points(..., timings=dict)` reports rectify/match/post. Node parameter `matcher` (default sgbm).
- `test/test_matcher.py` (VPI test skips off the Jetson). All 43 existing tests + 4 new pass
  (laptop: 43 + 3, VPI one skipped; drone: 4/4 with VPI).
- `tools/gpu_bench/`: capture_pairs.py (subscribe only), stereo_bench.py (accuracy / real / soak),
  live_ab.py (CPU, GPU, power, cuVSLAM rate while a method runs), topic_stats.py, vpi_profile.py.

## 1. Accuracy on exact ray-traced pairs (synthetic.py, skybeta-03 calibration, 27 deg mount)
Wall distance error (true 0.5 / 0.8 / 1.5 / 2.5 m), floor fit error, wall top:

| method | wall err (m) | floor (deg, m) worst | wall top 1.0 m seen at cam 1.2/1.3/1.45 |
|---|---|---|---|
| sgbm (flies now) | -0.005 -0.002 0.000 **-0.034** | 0.16, 0.004 | 1.05 1.05 1.05 |
| vpi-cuda (half res) | 0.005 0.004 0.003 -0.025 | 0.27, 0.002 | 1.10 1.05 1.05 |
| vpi-cuda-full (1024x640, 192 disp) | 0.003 0.003 0.000 **0.001** | 0.16, 0.002 | 1.05 1.05 1.05 |
| vpi-ofa (128 disp) | 0.010 0.008 0.008 -0.008 | 0.27, 0.001 | 1.10 1.05 1.05 |

All pass the mission's tolerances; full-res GPU is the most accurate (1 mm at 2.5 m).

## 2. Real frames: 40 live pairs, live stack running (steady state, uptime ~8 min)
Per frame, StereoRig.points() (median; CPU = process CPU time, all threads):

| method | total ms (p90) | rectify | match | post | CPU ms/frame | points | agree w/ SGBM (<=1 px) |
|---|---|---|---|---|---|---|---|
| sgbm (flies now) | 47.4 (63.6) | 4.6 | 36.3 | 2.6 | 136 | 15.0k | - |
| **vpi-cuda** | **21.8** (35.9) | 4.3 | **12.0** | 3.2 | 79 | 15.6k | 79 % |
| vpi-ofa | 50.5 (60.8) | 3.0 | 43.0 | 2.6 | 124 | 15.0k | 77 % |
| vpi-cuda-full | 81.0 (99.9) | 12.9 | 59.2 | 8.2 | 135 | 13.6k | 78 % |
| sgbm-full (reference) | 216.6 (262) | 9.3 | 197.4 | 7.3 | 498 | 12.7k | 91 % |

Floor fit 40/40 and "front clear" 40/40 for every method (same scene, same answer).
- GPU half-res: 2.2x faster per frame, matcher 3x faster.
- GPU full-res: 2.7x faster than CPU full-res, i.e. full resolution for less than CPU half-res x2.
- OFA: no gain on this board (VIC block-linear conversions + OFA ~= CPU time).
- Open: vpi-cuda still costs 79 ms CPU/frame; vpi_profile.py splits that (Y16 convert vs U8
  direct vs preallocated images vs sync). Interrupted by another session's reboot at 18:43.

## Pending (needs the lock back)
- vpi_profile.py (where the remaining CPU goes), live_ab.py soak (cuVSLAM rate, CPU, GPU, watts)
  and node-<method> side-by-side (achieved Hz and image age of a second stereo_depth instance).
- Drone /tmp is wiped on reboot: re-stage tools/gpu_bench + overwall_nav + the host vpi .so.
