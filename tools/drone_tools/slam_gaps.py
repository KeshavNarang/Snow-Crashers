#!/usr/bin/env python3
"""slam_gaps: how many camera frames does cuVSLAM actually lose, and does it line up with CPU load?

cuVSLAM logs a warning whenever the gap between two frames it receives is over 40 ms:
    [visual_slam_node]: Delta between current and previous frame [66.67 ms] is above threshold [40 ms]
The gaps are exact multiples of the 33.3 ms frame period, so each one counts frames that never arrived.
That makes cuVSLAM, the navigation-critical consumer, the best judge of the real delivered frame rate.
(A plain Python subscriber sees a lower rate, 24-26 Hz here, because of consumer-side drops; do not use it
to judge the camera. See agents/logs/2026-11-07-cpu-monitor.md.)

Run on the drone HOST:
    python3 slam_gaps.py                                  read the stack's slam tmux window
    python3 slam_gaps.py --file slam.log                  or a saved log
    python3 slam_gaps.py --cpu-log /tmp/cpu.jsonl         also correlate gaps with a cpu_watch --log file
Reads only.
"""
import argparse
import json
import math
import re
import subprocess
import time
from collections import Counter

GAP_RE = re.compile(r'\[(\d{9,10}\.\d+)\] \[visual_slam_node\]: Delta between current and previous frame '
                    r'\[([\d.]+) ms\]')
SLAM_STAMP_RE = re.compile(r'\[(\d{9,10}\.\d+)\] \[visual_slam_node\]')


def parse_gaps(text):
    """[(log time in epoch seconds, gap in ms)] for every cuVSLAM frame-gap warning in `text`."""
    return [(float(t), float(d)) for t, d in GAP_RE.findall(text)]


def summarise(gaps, span_s, fps=30.0, t0=None):
    """Frames lost and the effective rate over `span_s` seconds of log."""
    period = 1000.0 / fps
    hist, missed = Counter(), 0
    for _, d in gaps:
        k = max(int(round(d / period)), 2)         # gap in frame periods; a warning always means >= 1 lost
        hist[k] += 1
        missed += k - 1
    expected = max(span_s * fps, 1.0)
    loss = 100.0 * missed / expected
    start = t0 if t0 is not None else (gaps[0][0] if gaps else 0.0)
    minutes = max(int(math.ceil(span_s / 60.0)), 1)
    per_min = [0] * minutes
    for t, _ in gaps:
        i = int((t - start) // 60)
        if 0 <= i < minutes:
            per_min[i] += 1
    return {'warnings': len(gaps), 'missed': missed, 'expected': int(expected), 'loss_pct': round(loss, 2),
            'effective_hz': round(fps * (1 - loss / 100.0), 1), 'histogram': dict(sorted(hist.items())),
            'worst_gap_ms': max((d for _, d in gaps), default=0.0), 'per_minute': per_min}


def record_epoch(rec):
    """Epoch seconds of a cpu_watch --log record (older logs only have a local-time string)."""
    if rec.get('epoch') is not None:
        return rec['epoch']
    return time.mktime(time.strptime(rec['time'], '%Y-%m-%dT%H:%M:%S'))


def _top(rec):
    procs = rec.get('procs') or []
    return procs[0]['label'] if procs else None


def correlate(gaps, records, window=1.0):
    """Compare CPU state in the seconds nearest a frame gap with all the other seconds."""
    recs = sorted(((record_epoch(r), r) for r in records), key=lambda x: x[0])
    in_gap = set()
    for t, _ in gaps:
        best = min(range(len(recs)), key=lambda i: abs(recs[i][0] - t)) if recs else None
        if best is not None and abs(recs[best][0] - t) <= window:
            in_gap.add(best)
    gap_recs = [recs[i][1] for i in sorted(in_gap)]
    other = [r for i, (_, r) in enumerate(recs) if i not in in_gap]

    def mean(rs, key):
        v = [r.get(key, 0.0) for r in rs]
        return sum(v) / len(v) if v else 0.0

    out = {'gap_seconds': len(gap_recs), 'other_seconds': len(other),
           'cpu_in_gaps': mean(gap_recs, 'cpu_pct'), 'cpu_elsewhere': mean(other, 'cpu_pct'),
           'runq_in_gaps': mean(gap_recs, 'run_queue'), 'runq_elsewhere': mean(other, 'run_queue'),
           'top_in_gaps': Counter(filter(None, map(_top, gap_recs))).most_common(3),
           'top_elsewhere': Counter(filter(None, map(_top, other))).most_common(3)}
    dc, dq = out['cpu_in_gaps'] - out['cpu_elsewhere'], out['runq_in_gaps'] - out['runq_elsewhere']
    if out['gap_seconds'] < 3:
        out['verdict'] = 'not enough frame gaps inside the CPU log to say'
    elif dc >= 10 or dq >= 2:
        who = out['top_in_gaps'][0][0] if out['top_in_gaps'] else '?'
        out['verdict'] = (f'frame gaps coincide with higher load (CPU {dc:+.0f} points, run queue {dq:+.1f}); '
                          f'busiest process in those seconds: {who}')
    else:
        out['verdict'] = 'no relationship between frame gaps and CPU load found'
    return out


def log_span(text):
    stamps = [float(t) for t in SLAM_STAMP_RE.findall(text)]
    return (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--file', help='read this log instead of the stack tmux window')
    ap.add_argument('--cpu-log', help='cpu_watch --log file (JSON lines) to correlate against')
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    if a.file:
        text = open(a.file, errors='replace').read()
    else:
        text = subprocess.run(['docker', 'exec', 'skybeta-stack', 'tmux', 'capture-pane', '-p', '-J', '-S', '-30000',
                               '-t', 'skybeta:slam'], capture_output=True, text=True, check=True).stdout
    gaps = parse_gaps(text)
    s = summarise(gaps, log_span(text), a.fps)
    out = {'summary': s}
    if a.cpu_log:
        recs = [json.loads(l) for l in open(a.cpu_log) if l.strip()]
        out['correlation'] = correlate(gaps, recs)
    if a.json:
        print(json.dumps(out, indent=1, default=str))
        return
    print(f'{s["warnings"]} gap warnings in {log_span(text)/60:.1f} min of cuVSLAM log; {s["missed"]} of {s["expected"]} frames missing '
          f'= {s["loss_pct"]}% -> effective {s["effective_hz"]} Hz')
    print(f'gap size in frame periods -> count: {s["histogram"]}; worst gap {s["worst_gap_ms"]:.0f} ms')
    print(f'warnings per minute: {s["per_minute"]}')
    if 'correlation' in out:
        c = out['correlation']
        print(f'CPU log: {c["gap_seconds"]} seconds with a gap, {c["other_seconds"]} without')
        print(f'  CPU {c["cpu_in_gaps"]:.0f}% in gap seconds vs {c["cpu_elsewhere"]:.0f}% elsewhere; run queue '
              f'{c["runq_in_gaps"]:.1f} vs {c["runq_elsewhere"]:.1f}')
        print(f'  busiest process in gap seconds: {c["top_in_gaps"]}; elsewhere: {c["top_elsewhere"]}')
        print('  VERDICT: ' + c['verdict'])


if __name__ == '__main__':
    main()
