"""exposure_analysis: how does the camera cope when the light changes? (numpy only)

Pure functions, no ROS and no OpenCV, so they are testable on any machine (test_exposure_analysis.py) and
run the same on the drone and a laptop.

  frame_metrics(img)   brightness, clipping, sharpness and texture of one 8-bit grey frame
  find_steps(t, y)     abrupt brightness changes (lens covered / uncovered, tunnel mouth)
  settle_time(...)     how long until brightness stays within a band of its final value
  recover_time(...)    how long until a quantity (e.g. texture) stays above a fraction of a TARGET
  summarise(frames)    baseline + per-event settle time, over-exposure, darkest point, texture recovery
  format_report(...)   the summary as text

Terms
  mean / p5 / p50 / p95   grey levels 0-255 of the frame
  sat    % of pixels at 250 or above (clipped bright: no information left in them)
  black  % of pixels at 5 or below (clipped dark)
  sharp  variance of the Laplacian (high = fine detail; blur and flat scenes are low)
  edges  % of pixels whose neighbour differs by more than 20 grey levels (a proxy for how much texture the
         stereo matcher and visual odometry have to work with)
"""
import json

import numpy as np

SAT_LEVEL = 250
BLACK_LEVEL = 5
EDGE_STEP = 20


def frame_metrics(img):
    """Metrics of one frame: a 2-D uint8 array."""
    a = np.asarray(img)
    if a.ndim != 2 or a.dtype != np.uint8:
        raise ValueError(f'need a 2-D uint8 grey image, got shape {a.shape} dtype {a.dtype}')
    p5, p50, p95 = np.percentile(a, (5, 50, 95))
    i = a.astype(np.int16)
    gx = np.abs(np.diff(i, axis=1))[:-1, :]                 # (h-1, w-1)
    gy = np.abs(np.diff(i, axis=0))[:, :-1]
    edges = 100.0 * float(np.mean(np.maximum(gx, gy) > EDGE_STEP)) if gx.size else 0.0
    f = a.astype(np.float64)
    lap = 4 * f[1:-1, 1:-1] - f[:-2, 1:-1] - f[2:, 1:-1] - f[1:-1, :-2] - f[1:-1, 2:] if min(a.shape) > 2 else np.zeros(1)
    return {'mean': float(a.mean()), 'p5': float(p5), 'p50': float(p50), 'p95': float(p95),
            'sat': 100.0 * float(np.mean(a >= SAT_LEVEL)), 'black': 100.0 * float(np.mean(a <= BLACK_LEVEL)),
            'sharp': float(lap.var()), 'edges': edges}


def find_steps(t, y, window_s=0.5, min_abs=25.0, min_ratio=1.8, merge_s=1.5):
    """Abrupt changes in y(t): the median of the next `window_s` against the median of the previous one.
    A step needs BOTH an absolute change of `min_abs` grey levels and a `min_ratio` change in ratio, so slow
    drift, noise, and the tail of an exposure recovery are not counted. Nearby detections merge into one."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    if len(t) < 4:
        return []
    dt = float(np.median(np.diff(t)))
    k = max(int(round(window_s / dt)), 2)
    if len(t) < 2 * k:
        return []
    cand = []
    for i in range(k, len(t) - k + 1):
        before, after = float(np.median(y[i - k:i])), float(np.median(y[i:i + k]))
        lo, hi = min(before, after), max(before, after)
        if abs(after - before) >= min_abs and hi / max(lo, 1.0) >= min_ratio:
            cand.append((i, before, after))
    events, group = [], []
    for c in cand:
        if group and t[c[0]] - t[group[-1][0]] > merge_s:
            events.append(group)
            group = []
        group.append(c)
    if group:
        events.append(group)
    out = []
    for g in events:
        i, before, after = max(g, key=lambda c: abs(c[2] - c[1]))
        out.append({'t': float(t[i]), 'before': before, 'after': after,
                    'direction': 'dark' if after < before else 'bright'})
    return out


def _tail(t, y, t0, t_end):
    """Samples in [t0, t_end] and the median of the last 20 % of that window (at least 2 s): its final value."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    m = (t >= t0) & (t <= t_end)
    tt, yy = t[m], y[m]
    if len(tt) < 3:
        return tt, yy, None
    tail = max(2.0, 0.2 * (t_end - t0))
    return tt, yy, float(np.median(yy[tt >= t_end - tail]))


def settle_time(t, y, t0, t_end, tol=0.10, floor=3.0):
    """Seconds after t0 until y stays within +-tol of its final value (or `floor` grey levels, whichever is
    larger) for the rest of the window. 0.0 if it never left the band, None if it is still outside it at the end."""
    tt, yy, final = _tail(t, y, t0, t_end)
    if final is None:
        return None
    outside = np.abs(yy - final) > max(tol * abs(final), floor)
    if not outside.any():
        return 0.0
    j = int(np.nonzero(outside)[0][-1])
    return None if j == len(tt) - 1 else float(tt[j + 1] - t0)


def recover_time(t, y, t0, t_end, target, frac=0.5):
    """Seconds after t0 until y stays at or above frac*target for the rest of the window (0.0 if it always did,
    None if it is still below at the end). `target` is the pre-event baseline, never the series' own end state:
    a camera that collapses and stays collapsed has not recovered."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    m = (t >= t0) & (t <= t_end)
    tt, yy = t[m], y[m]
    if len(tt) < 2:
        return None
    below = yy < frac * target
    if not below.any():
        return 0.0
    j = int(np.nonzero(below)[0][-1])
    return None if j == len(tt) - 1 else float(tt[j + 1] - t0)


def summarise(frames, side=None):
    """Baseline and per-event behaviour from a list of frame dicts (frame_metrics fields plus 't', 'side')."""
    fr = [f for f in frames if side is None or f.get('side') == side]
    fr.sort(key=lambda f: f['t'])
    if len(fr) < 10:
        raise ValueError(f'need at least 10 frames to say anything, got {len(fr)}')
    t = np.array([f['t'] for f in fr])
    col = lambda k: np.array([f.get(k, 0.0) for f in fr], float)
    mean, sat, black, edges = col('mean'), col('sat'), col('black'), col('edges')
    steps = find_steps(t, mean)
    pre = t < (steps[0]['t'] - 0.5) if steps else np.ones(len(t), bool)
    if pre.sum() < 3:
        pre = np.ones(len(t), bool)
    baseline = {k: float(col(k)[pre].mean()) for k in ('mean', 'sat', 'black', 'sharp', 'edges')}
    dt = float(np.median(np.diff(t)))
    events = []
    for i, e in enumerate(steps):
        end = steps[i + 1]['t'] if i + 1 < len(steps) else float(t[-1])
        win = (t >= e['t']) & (t <= end)
        first3 = (t >= e['t']) & (t <= e['t'] + 3.0)
        events.append({**e, 'end': end,
                       'settle_s': settle_time(t, mean, e['t'], end),
                       'peak_sat_pct': float(sat[first3].max()) if first3.any() else 0.0,
                       'min_mean': float(mean[win].min()),
                       'black_s': float(np.sum(black[win] >= 50.0) * dt),
                       'edges_min': float(edges[win].min()),
                       'texture_recover_s': recover_time(t, edges, e['t'], end, baseline['edges'], 0.5)})
    return {'n_frames': len(fr), 'duration_s': float(t[-1] - t[0]), 'baseline': baseline, 'events': events}


def _s(v, unit='s'):
    return 'never (still changing at the end of the window)' if v is None else f'{v:.1f} {unit}'


def format_report(s):
    b = s['baseline']
    lines = [f'{s["n_frames"]} frames over {s["duration_s"]:.0f} s',
             f'baseline (before the first change): mean {b["mean"]:.0f}/255, saturated {b["sat"]:.1f}%, black {b["black"]:.1f}%, '
             f'sharpness {b["sharp"]:.0f}, texture {b["edges"]:.0f}%']
    if not s['events']:
        lines.append('no brightness changes found: the scene was steady, so nothing about exposure recovery can be said')
        return '\n'.join(lines)
    for e in s['events']:
        lines.append(f'{e["direction"].upper()} at t={e["t"]:.1f} s: mean {e["before"]:.0f} -> {e["after"]:.0f}')
        lines.append(f'  brightness settles (within 10 % of its final value) after {_s(e["settle_s"])}')
        lines.append(f'  darkest mean {e["min_mean"]:.0f}; frame mostly black for {e["black_s"]:.1f} s; '
                     f'saturated up to {e["peak_sat_pct"]:.0f}% of pixels in the first 3 s')
        lines.append(f'  texture fell to {e["edges_min"]:.0f}% (baseline {b["edges"]:.0f}%); back above half of baseline after '
                     f'{_s(e["texture_recover_s"])}')
    return '\n'.join(lines)


def load_frames(path):
    """(frames, skipped): JSON-lines file written by exposure_probe.py. Blank and broken lines are skipped."""
    frames, skipped = [], 0
    with open(path) as f:
        for line in f:
            if not line.strip():
                continue
            try:
                frames.append(json.loads(line))
            except ValueError:
                skipped += 1
    return frames, skipped
