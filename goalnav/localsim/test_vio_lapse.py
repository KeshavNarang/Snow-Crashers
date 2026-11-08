"""The simulated PX4 estimate can lose visual odometry for a while, as skybeta-03 did.

Observation (skybeta-03, 2026-11-08 01:31, goalnav running): PX4 fused vision only 60 % of a 30 s
sample (estimator cs_ev_pos), vision reaching it 236-400 ms old. Without fusion PX4 dead-reckons on
the IMU and its position walks off; when vision returns the estimator resets to it. The simulator's
estimate only drifted slowly, so no run ever met a lapse.

Run from goalnav/: python -m pytest localsim/test_vio_lapse.py
"""
import numpy as np

from localsim import fakeros as m
from localsim.vehicle import Vehicle, VehicleConfig


def _vehicle(**cfg):
    v = Vehicle(m.Bus({}), (0., 0., 0.), 0., VehicleConfig(**cfg), seed=3)
    v.landed = False
    return v


def test_off_by_default_no_lapse():
    v = _vehicle()
    for _ in range(3000):                       # 60 s at 50 Hz
        v.lapse_step(0.02)
    assert v.lapses == [] and float(np.linalg.norm(v.lapse)) == 0.


def test_lapses_walk_the_estimate_off_then_reset():
    v = _vehicle(vio_lapse_per_min=6., vio_lapse_s=(1.5, 1.5))
    worst, t = 0., 0.
    while t < 60.:
        v.bus.ns = int(t * 1e9)
        v.lapse_step(0.02)
        worst = max(worst, float(np.linalg.norm(v.lapse)))
        t += 0.02
    assert len(v.lapses) >= 2                   # about 6 a minute
    assert worst > 0.05                         # dead reckoning walks off by centimetres to decimetres
    if not v.in_lapse:
        assert float(np.linalg.norm(v.lapse)) == 0.     # vision back: the estimate resets to it
