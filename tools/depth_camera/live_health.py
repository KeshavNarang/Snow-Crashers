"""live_health: are the OAK cameras, cuVSLAM and (optionally) on-device depth healthy right now?

assess() is the pure decision (tested in test_live_health.py); measure() collects the inputs on the drone from
small topics only: /oak/{left,right}/camera_info and /oak/stereo/camera_info (one per frame, header stamps) and
/visual_slam/status (vo_state: 1 = tracking). Used by driver_swap.py to decide whether to keep a new driver
configuration or revert it. Read-only.
"""
import time

import depth_analysis as da

LIMITS = {'min_hz': 25.0, 'max_missing_pct': 3.0, 'min_pair': 0.95, 'pair_tol_s': 0.005, 'min_tracking': 0.9,
          'min_depth_hz': 15.0}


def assess(m, want_depth, limits=LIMITS):
    """m: {'left': [stamps], 'right': [stamps], 'depth': [stamps], 'vo_states': [ints], 'window_s': s}"""
    fails, numbers = [], {}
    for side in ('left', 'right'):
        st = da.stamp_stats(m.get(side) or [], 30.0)
        numbers[side] = st
        if st is None:
            fails.append(f'no {side} camera frames')
            continue
        if st['rate_hz'] < limits['min_hz']:
            fails.append(f'{side} camera at {st["rate_hz"]:.1f} Hz')
        if st['missing_pct'] > limits['max_missing_pct']:
            fails.append(f'{side} camera {st["missing_pct"]:.1f} % frames missing')
    if numbers.get('left') and numbers.get('right'):
        pf = da.pair_fraction(m['left'], m['right'], limits['pair_tol_s'])
        numbers['pair_fraction'] = pf
        if pf < limits['min_pair']:
            fails.append(f'only {pf:.0%} of left frames pair with a right frame')
    else:
        numbers['pair_fraction'] = None
    states = list(m.get('vo_states') or [])
    if not states:
        numbers['tracking_fraction'] = None
        fails.append('cuVSLAM silent (no /visual_slam/status)')
    else:
        tf = sum(1 for s in states if s == 1) / len(states)
        numbers['tracking_fraction'] = tf
        if states[-1] != 1 or tf < limits['min_tracking']:
            fails.append(f'cuVSLAM not tracking (last state {states[-1]}, {tf:.0%} tracking)')
    st = da.stamp_stats(m.get('depth') or [], 30.0)
    numbers['depth'] = st
    if want_depth:
        if st is None:
            fails.append('no depth frames')
        elif st['rate_hz'] < limits['min_depth_hz']:
            fails.append(f'depth at {st["rate_hz"]:.1f} Hz')
    return {'ok': not fails, 'why': '; '.join(fails), 'numbers': numbers}


def measure(want_depth, wait_s=40.0, window_s=8.0):
    """Collect for up to wait_s seconds; return as soon as the last window_s seconds look healthy."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

    rclpy.init()
    node = Node('depth_swap_health')
    got = {'left': [], 'right': [], 'depth': [], 'vo_states': []}      # (arrival, value)

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    for key, topic in (('left', '/oak/left/camera_info'), ('right', '/oak/right/camera_info'),
                       ('depth', '/oak/stereo/camera_info')):
        node.create_subscription(CameraInfo, topic, lambda m, k=key: got[k].append((time.time(), stamp(m))),
                                 qos_profile_sensor_data)
    node.create_subscription(VisualSlamStatus, '/visual_slam/status',
                             lambda m: got['vo_states'].append((time.time(), int(m.vo_state))), 10)
    t0, last_eval, result = time.time(), 0.0, {'ok': False, 'why': 'no measurement yet', 'numbers': {}}
    try:
        while time.time() - t0 < wait_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - t0 >= window_s and now - last_eval >= 1.0:
                last_eval = now
                win = {k: [v for (a, v) in got[k] if a >= now - window_s] for k in got}
                win['window_s'] = window_s
                result = assess(win, want_depth)
                if result['ok']:
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result['waited_s'] = round(time.time() - t0, 1)
    return result
