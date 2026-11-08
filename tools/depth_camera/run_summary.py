"""run_summary: one recording -> the quantities acceptance.judge() is defined on.

A recording is raw observations: (arrival, header stamp) pairs per stream, cuVSLAM states, odometry positions,
camera_info, TF, and one decoded depth snapshot. Quantities that cannot be computed are left out, so the contract
reports them as "not measured" instead of passing on an invented value. Host-side numbers (cuVSLAM gap warnings
from slam_gaps.py, CPU from cpu_watch.py) are merged in.
"""
import numpy as np

import depth_analysis as da


def _stamps(pairs):
    return [s for (_, s) in pairs]


def cpu_from_watch(cw, saturated_pct=90.0):
    """cpu_watch.py --json -> {'total_cores', 'saturated_cores'}"""
    return {'total_cores': round(cw['cpu_pct'] / 100.0 * cw['ncpu'], 3),
            'saturated_cores': sum(1 for c in cw['core_pct'] if c >= saturated_pct)}


def summarise(rec, host=None, nominal_hz=30.0):
    out = {}
    for side in ('left', 'right'):
        st = da.stamp_stats(_stamps(rec.get(side) or []), nominal_hz)
        if st is not None:
            out[side] = st
    if 'left' in out and 'right' in out:
        out['pair_fraction'] = da.pair_fraction(_stamps(rec['left']), _stamps(rec['right']), 0.005)
    states = rec.get('vo_states') or []
    if states:
        out['tracking_fraction'] = sum(1 for s in states if s == 1) / len(states)
    odom = rec.get('odom') or []
    if len(odom) >= 2:
        t = np.array([o[0] for o in odom], float)
        p = np.array([o[1] for o in odom], float)
        out['odom_rate_hz'] = float((len(t) - 1) / (t.max() - t.min()))
        out['static_drift_m'] = float(np.max(np.linalg.norm(p - p[0], axis=1)))
    infos = rec.get('camera_info') or {}
    if 'left' in infos and 'right' in infos:
        out['camera_info'] = {'left': infos['left'], 'right': infos['right']}
    if rec.get('tf'):
        out['tf'] = rec['tf']
    img, info, depth = rec.get('depth_image'), infos.get('depth'), rec.get('depth') or []
    if img is not None and info is not None and depth:
        d = img['metres']
        k = info['k']
        ages = np.array([a - s for (a, s) in depth], float) * 1000.0
        out['depth'] = {'stream': da.stamp_stats(_stamps(depth), nominal_hz), 'encoding': img['encoding'],
                        'size': (img['width'], img['height']), 'info_size': (info['width'], info['height']),
                        'age_p95_ms': float(np.percentile(ages, 95)), 'valid_fraction': da.valid_fraction(d),
                        'floor': da.floor_from_depth(d, (k[0], k[4], k[2], k[5])),
                        'expected_pitch_deg': rec.get('expected_pitch_deg')}
    for key, value in (host or {}).items():
        out[key] = value
    return out
