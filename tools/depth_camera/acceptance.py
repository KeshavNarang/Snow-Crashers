"""acceptance: the pass/fail contract for on-device depth, fixed before any measurement.

judge(baseline, candidate) compares a run with the depth configuration against a run with the organizers'
configuration taken minutes earlier on the same scene. Rules (agents/logs/2026-11-07-depth-camera-plan.md):

  A1 cameras   left/right >= 29.5 Hz by header stamp, missing frames <= baseline + 0.5 points, pairing >= 99 %
  A2 cuVSLAM   gap warnings/min <= 1.5 x baseline + 2, odometry >= 29 Hz, tracking >= 99 %,
               static drift <= max(2 cm, 2 x baseline)
  A3 geometry  left/right camera_info identical; TF base_link -> optical frames within 1 mm and 0.1 deg
  A4 CPU       no saturated core; total CPU <= baseline + 0.5 core
  B1 depth     >= 25 Hz, 16UC1, image size = its camera_info size, age p95 <= 100 ms
  B2 frame     against a host stereo match of the same instant: median |error| <= 3 %, best shift 0 px
  B3 floor     when the floor dominates (inliers >= 50 %): pitch within 2 deg of the mount pitch; else skipped
  B4 coverage  valid fraction >= the host stereo match's

A criterion whose inputs are missing is 'not measured' and fails the verdict; only B3 may be 'skipped'.
"""
import depth_analysis as da


class Missing(Exception):
    pass


def _get(d, *path):
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur or cur[p] is None:
            raise Missing('.'.join(str(x) for x in path))
        cur = cur[p]
    return cur


def _a1(b, c):
    bad = []
    for side in ('left', 'right'):
        rate, miss = _get(c, side, 'rate_hz'), _get(c, side, 'missing_pct')
        base_miss = _get(b, side, 'missing_pct')
        if rate < 29.5:
            bad.append(f'{side} {rate:.2f} Hz < 29.5')
        if miss > base_miss + 0.5:
            bad.append(f'{side} missing {miss:.2f} % > baseline {base_miss:.2f} + 0.5')
    pf = _get(c, 'pair_fraction')
    if pf < 0.99:
        bad.append(f'pairing {pf:.3f} < 0.99')
    return bad


def _a2(b, c):
    bad = []
    gw, bgw = _get(c, 'gap_warnings_per_min'), _get(b, 'gap_warnings_per_min')
    if gw > 1.5 * bgw + 2:
        bad.append(f'gap warnings {gw:.1f}/min > 1.5 x {bgw:.1f} + 2')
    if _get(c, 'odom_rate_hz') < 29.0:
        bad.append(f'odometry {c["odom_rate_hz"]:.2f} Hz < 29')
    if _get(c, 'tracking_fraction') < 0.99:
        bad.append(f'tracking {c["tracking_fraction"]:.3f} < 0.99')
    drift, bdrift = _get(c, 'static_drift_m'), _get(b, 'static_drift_m')
    if drift > max(0.02, 2 * bdrift):
        bad.append(f'static drift {drift * 100:.1f} cm > max(2 cm, 2 x {bdrift * 100:.1f} cm)')
    return bad


def _a3(b, c):
    bad = []
    for side in ('left', 'right'):
        diff = da.camera_info_diff(_get(b, 'camera_info', side), _get(c, 'camera_info', side))
        if diff:
            bad.append(f'{side} camera_info changed: {diff}')
        dist, ang = da.tf_diff(_get(b, 'tf', side), _get(c, 'tf', side))
        if dist > 0.001 or ang > 0.1:
            bad.append(f'{side} TF moved {dist * 1000:.1f} mm / {ang:.2f} deg')
    return bad


def _a4(b, c):
    bad = []
    if _get(c, 'cpu', 'saturated_cores') > 0:
        bad.append(f'{c["cpu"]["saturated_cores"]} saturated core(s)')
    tot, btot = _get(c, 'cpu', 'total_cores'), _get(b, 'cpu', 'total_cores')
    if tot > btot + 0.5:
        bad.append(f'CPU {tot:.2f} cores > baseline {btot:.2f} + 0.5')
    return bad


def _b1(b, c):
    bad = []
    rate = _get(c, 'depth', 'stream', 'rate_hz')
    if rate < 25.0:
        bad.append(f'depth {rate:.1f} Hz < 25')
    if _get(c, 'depth', 'encoding') != '16UC1':
        bad.append(f'encoding {c["depth"]["encoding"]} is not 16UC1')
    if tuple(_get(c, 'depth', 'size')) != tuple(_get(c, 'depth', 'info_size')):
        bad.append(f'image {tuple(c["depth"]["size"])} != camera_info {tuple(c["depth"]["info_size"])}')
    if _get(c, 'depth', 'age_p95_ms') > 100.0:
        bad.append(f'age p95 {c["depth"]["age_p95_ms"]:.0f} ms > 100')
    return bad


def _b2(b, c):
    bad = []
    med, shift = _get(c, 'depth', 'vs_host', 'median_rel'), _get(c, 'depth', 'vs_host', 'best_shift')
    if med > 0.03:
        bad.append(f'median error vs host {med:.1%} > 3 %')
    if shift != 0:
        bad.append(f'best horizontal shift {shift} px: depth is not in the frame its camera_info claims')
    return bad


def _b4(b, c):
    v, hv = _get(c, 'depth', 'valid_fraction'), _get(c, 'depth', 'host_valid_fraction')
    return [] if v >= hv else [f'valid {v:.0%} < host stereo {hv:.0%}']


RULES = [('A1', _a1), ('A2', _a2), ('A3', _a3), ('A4', _a4)]
DEPTH_RULES = [('B1', _b1), ('B2', _b2), ('B4', _b4)]


def _row(cid, fn, b, c):
    try:
        bad = fn(b, c)
    except Missing as e:
        return {'id': cid, 'pass': False, 'status': 'not measured', 'detail': f'missing {e}'}
    return {'id': cid, 'pass': not bad, 'status': 'pass' if not bad else 'fail', 'detail': '; '.join(bad) or 'ok'}


def _b3(c):
    fl = (c.get('depth') or {}).get('floor')
    exp = (c.get('depth') or {}).get('expected_pitch_deg')
    if not fl or exp is None or fl.get('inlier_frac', 0) < 0.5:
        return {'id': 'B3', 'pass': True, 'status': 'skipped', 'detail': 'floor not dominant in view'}
    err = abs(fl['pitch_deg'] - exp)
    ok = err <= 2.0
    return {'id': 'B3', 'pass': ok, 'status': 'pass' if ok else 'fail',
            'detail': f'floor pitch {fl["pitch_deg"]:.1f} deg vs mount {exp:.1f} deg'}


def judge(baseline, candidate, want_depth=True):
    rows = [_row(cid, fn, baseline, candidate) for cid, fn in RULES]
    if want_depth:
        rows += [_row(cid, fn, baseline, candidate) for cid, fn in DEPTH_RULES]
        rows.append(_b3(candidate))
        rows.sort(key=lambda r: r['id'])
    return {'pass': all(r['pass'] for r in rows), 'rows': rows}
