#!/usr/bin/env python3
"""Summarise one goalnav flight log folder (as tools/fly logs pulls it), in a minute.

    python3 tools/postflight.py flight-logs/<date-time>        (needs: pip install rosbags)

Reads run.json and the rosbag (bag/, MCAP) and prints: how the mission went (goalnav states
and events), PX4 (link, armed, mode, VIO fusion, height), sensor rates (Jetson depth, colour,
cuVSLAM) and node errors from /rosout. Writes summary.json next to the bag. Images are skipped.
"""
import argparse
import json
from pathlib import Path
import sys

RATES = {'/goalnav/stereo/points': 'depth (Jetson)', '/oak/rgb/image_raw': 'colour',
         '/visual_slam/tracking/odometry': 'cuVSLAM', '/goalnav/status': 'goalnav status',
         '/skybeta/setpoint/twist': 'setpoints'}
READ = {'/goalnav/status', '/skybeta/state', '/fmu/out/estimator_status_flags',
        '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_land_detected', '/rosout'}


def changes(samples):
    """[(t, value)] -> only the samples where the value changed."""
    out, last = [], object()
    for t, v in samples:
        if v != last:
            out.append((t, v))
            last = v
    return out


def summarise(run):
    try:
        from rosbags.highlevel import AnyReader
    except ImportError:
        sys.exit('needs rosbags: pip install rosbags')
    bag = run / 'bag'
    if not bag.exists():
        sys.exit(f'no bag/ in {run} (was record:=false?)')
    counts, first, last = {}, None, None
    status, state, fused, local, landed, errors = [], [], [], [], [], []
    with AnyReader([bag]) as reader:
        wanted = [c for c in reader.connections if c.topic in READ or c.topic in RATES]
        for conn, stamp, raw in reader.messages(connections=wanted):
            t = stamp * 1e-9
            first = t if first is None else first
            last = t
            counts[conn.topic] = counts.get(conn.topic, 0) + 1
            if conn.topic not in READ:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/goalnav/status':
                status.append((t, json.loads(msg.data)))
            elif conn.topic == '/skybeta/state':
                s = json.loads(msg.data)
                state.append((t, (s.get('px4_link'), s.get('armed'), s.get('nav_state_name'))))
            elif conn.topic == '/fmu/out/estimator_status_flags':
                fused.append((t, bool(msg.cs_ev_pos)))
            elif conn.topic == '/fmu/out/vehicle_local_position':
                local.append((t, -float(msg.z), bool(msg.xy_valid), float(msg.dist_bottom)))
            elif conn.topic == '/fmu/out/vehicle_land_detected':
                landed.append((t, bool(msg.landed)))
            elif conn.topic == '/rosout' and msg.level >= 40:      # ERROR, FATAL
                errors.append((t, msg.name, msg.msg[:160]))
    if first is None:
        sys.exit('the bag has none of the expected topics')
    span = max(last - first, 1e-6)
    rel = lambda t: round(t - first, 2)                            # noqa: E731
    final = status[-1][1] if status else {}
    return {
        'run': run.name,
        'args': json.loads((run / 'run.json').read_text()).get('args', {}) if (run / 'run.json').exists() else {},
        'duration_s': round(span, 1),
        'goalnav_states': [(rel(t), s) for t, s in changes((t, st.get('state')) for t, st in status)],
        'goalnav_final': {k: final.get(k) for k in ('state', 'reason', 'goal', 'pad')},
        'goalnav_events': final.get('events', []),
        'px4_link_armed_mode': [(rel(t), v) for t, v in changes(state)],
        'vio_fused': [(rel(t), v) for t, v in changes(fused)],
        'landed': [(rel(t), v) for t, v in changes(landed)],
        'max_height_m': round(max((h for _, h, _, _ in local), default=0.), 2),
        'xy_valid_share': round(sum(v for _, _, v, _ in local) / len(local), 3) if local else None,
        'rates_hz': {RATES[k]: round(counts.get(k, 0) / span, 2) for k in RATES},
        'errors': [(rel(t), n, m) for t, n, m in errors[:20]],
        'error_count': len(errors),
    }


def show(s):
    print(f"== {s['run']}: {s['duration_s']} s, profile {s['args'].get('profile', '?')}, camera {s['args'].get('camera', '?')}")
    print('-- goalnav:', ' -> '.join(f'{v}@{t}' for t, v in s['goalnav_states']) or 'no status')
    print('   final:', s['goalnav_final'])
    for e in s['goalnav_events'][-15:]:
        print('   event:', e)
    print('-- PX4 (link, armed, mode):', s['px4_link_armed_mode'][-8:])
    print('   VIO fused (cs_ev_pos):', s['vio_fused'], ' xy_valid share:', s['xy_valid_share'])
    print('   landed:', s['landed'], ' max height:', s['max_height_m'], 'm')
    print('-- rates (Hz):', s['rates_hz'])
    print(f"-- errors in /rosout: {s['error_count']}")
    for e in s['errors'][:8]:
        print('  ', e)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('run', type=Path, help='flight-logs/<date-time> (as tools/fly logs saves it)')
    run = p.parse_args().run
    s = summarise(run)
    show(s)
    (run / 'summary.json').write_text(json.dumps(s, indent=1, default=str))
    print(f'saved {run / "summary.json"}')


if __name__ == '__main__':
    main()
