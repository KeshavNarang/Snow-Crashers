"""Pre-flight sensor checks: judge a few seconds of statistics (no ROS: tested on its own).

Everything the over-the-wall mission depends on, with the drone on the ground:

    camera        both mono streams at rate, both CameraInfo, the stereo TF (baseline)
    camera tilt   the real mount pitch, measured two ways (floor-plane fit from stereo, IMU
                  gravity), against the expected 27 deg AND against what the stack's TF claims
    depth         stereo_depth running, enough of the image matching, no phantom obstacle
    IMU           rate and gravity magnitude
    VIO           cuVSLAM odometry rate and tracking state
    PX4           link, valid local position, landed and disarmed, height offset vs vision

evaluate(stats) -> [(status, name, detail)], status 'PASS' | 'WARN' | 'FAIL'. Any FAIL: don't fly.
"""
import math


def _rate(stats, key):
    return stats.get(key, 0) / max(1e-6, stats.get('duration', 1.0))


def evaluate(stats, expected_pitch=27.0, pitch_tolerance=4.0):
    out = []

    def add(status, name, detail):
        out.append((status, name, detail))

    # -- camera --------------------------------------------------------------
    for side in ('left', 'right'):
        r = _rate(stats, side + '_images')
        add('PASS' if r >= 20 else 'WARN' if r >= 5 else 'FAIL', f'camera {side}', f'{r:.1f} Hz (30 expected)')
    infos = [s for s in ('left', 'right') if stats.get(s + '_info')]
    add('PASS' if len(infos) == 2 else 'FAIL', 'camera calibration', f'CameraInfo from {", ".join(infos) or "neither"}')
    b = stats.get('baseline_m')
    if b is None:
        add('WARN', 'stereo TF', 'no left -> right TF: depth falls back to the CameraInfo baseline, no rotation')
    else:
        add('PASS' if 0.07 <= b <= 0.08 else 'FAIL', 'stereo TF', f'baseline {b * 100:.1f} cm (7.5 expected)')

    # -- depth -----------------------------------------------------------------
    fr = _rate(stats, 'fronts')
    add('PASS' if fr >= 5 else 'WARN' if fr >= 1 else 'FAIL', 'stereo_depth',
        f'{fr:.1f} Hz' + ('' if fr else ' (is it running? ros2 run overwall_nav stereo_depth)'))
    if fr:
        v = stats.get('valid_median', 0.0)
        add('PASS' if v >= 0.15 else 'WARN' if v >= 0.05 else 'FAIL', 'stereo matching',
            f'{v * 100:.0f}% of the image matched (low: dark, blank floor, covered lens)')
        ms = stats.get('proc_ms_median', 0.0)
        add('PASS' if ms <= 60 else 'WARN', 'depth processing', f'{ms:.0f} ms per frame, images '
            f'{stats.get("age_ms_median", 0.0):.0f} ms old')
        front = stats.get('front_on_ground')
        add('PASS' if front is None else 'WARN', 'obstacle check',
            'nothing in front at drone height' if front is None else
            f'something {front:.2f} m ahead at drone height: is the take-off spot clear?')

    # -- camera tilt -------------------------------------------------------------------
    # The IMU (gravity) is the reference: it does not depend on what the camera happens to see.
    # The floor fit needs a big flat floor in view; when it disagrees it is the scene, not the mount.
    floor_pitch, imu_pitch = stats.get('floor_pitch_deg'), stats.get('imu_pitch_deg')
    if imu_pitch is not None:
        ok = abs(imu_pitch - expected_pitch) <= pitch_tolerance
        add('PASS' if ok else 'FAIL', 'camera tilt (IMU)',
            f'{imu_pitch:.1f} deg down from gravity (expected {expected_pitch:.0f}; drone level and still)')
    else:
        add('WARN', 'camera tilt (IMU)', 'no IMU reading')
    reference = imu_pitch if imu_pitch is not None else expected_pitch
    if floor_pitch is None:
        add('WARN', 'camera tilt (floor)', 'no floor fit (no depth, or not enough flat floor in view)')
    elif abs(floor_pitch - reference) <= pitch_tolerance:
        add('PASS', 'camera tilt (floor)',
            f'{floor_pitch:.1f} deg down, {stats.get("floor_height_m", 0):.2f} m above the floor')
    else:
        add('WARN', 'camera tilt (floor)',
            f'floor fit {floor_pitch:.1f} deg disagrees with {reference:.1f}: not enough flat floor in view '
            '(the mission then ignores it and uses camera_height_m)')
    measured = imu_pitch if imu_pitch is not None else floor_pitch
    tf_pitch = stats.get('tf_pitch_deg')
    if tf_pitch is not None and measured is not None:
        if abs(tf_pitch - measured) > 5.0:
            add('WARN', 'stack camera TF', f'TF says the camera is {tf_pitch:.1f} deg down, it is {measured:.1f}: '
                "cuVSLAM's frame is tilted by the difference, so climbs show up as fake forward/back motion. "
                'Set CAMERA_PITCH (radians, + = down) in /etc/skybeta/drone.env')
        else:
            add('PASS', 'stack camera TF', f'TF {tf_pitch:.1f} deg agrees with the measured {measured:.1f}')

    # -- IMU -------------------------------------------------------------------
    ir = _rate(stats, 'imu')
    add('PASS' if ir >= 100 else 'WARN' if ir >= 20 else 'FAIL', 'IMU', f'{ir:.0f} Hz (~200 expected)')
    g = stats.get('imu_g')
    if g is not None:
        add('PASS' if 9.3 <= g <= 10.3 else 'WARN', 'IMU gravity', f'{g:.2f} m/s^2 (still drone: 9.81)')

    # -- VIO -------------------------------------------------------------------
    orr = _rate(stats, 'odom')
    add('PASS' if orr >= 20 else 'WARN' if orr >= 5 else 'FAIL', 'cuVSLAM odometry', f'{orr:.1f} Hz (30 expected)')
    vo = stats.get('vo_state')
    if vo is not None:
        add('PASS' if vo == 1 else 'FAIL', 'cuVSLAM tracking', 'tracking' if vo == 1 else f'vo_state {vo} (1 = tracking)')

    # -- PX4 (via flight_interface) ------------------------------------------------------
    state = stats.get('state')
    if not state:
        add('FAIL', 'flight_interface', 'no /skybeta/state: the stack or our launch file is not running')
        return out
    add('PASS' if state.get('px4_link') else 'FAIL', 'PX4 link', 'connected' if state.get('px4_link') else
        'no vehicle_status: battery / cable (host/doctor.sh uart-traffic)')
    local = state.get('local_enu')
    add('PASS' if local else 'FAIL', 'PX4 position', 'valid' if local else 'no valid local position (VIO not fused?)')
    add('PASS' if state.get('landed') else 'WARN', 'landed', 'on the ground' if state.get('landed') else
        'PX4 does not report landed: takeover only happens on the ground')
    add('PASS' if not state.get('armed') else 'WARN', 'disarmed', 'disarmed' if not state.get('armed') else 'ARMED')
    origin = state.get('origin_enu')
    if local and origin:
        dz = float(local[2]) - float(origin[2])
        add('PASS' if abs(dz) <= 0.2 else 'WARN', 'PX4 vs vision height',
            f'{dz:+.2f} m (the interface floors poses 0.3 m above the vision origin: low targets get lifted)')
    return out


def verdict(results):
    if any(s == 'FAIL' for s, _, _ in results):
        return 'NOT READY'
    return 'READY (with warnings)' if any(s == 'WARN' for s, _, _ in results) else 'READY'


def oak_imu_pitch_deg(accel_imu):
    """Camera pitch DOWN from a still OAK-D-S2 IMU (BNO086), in the IMU's own axes. The driver's
    TF says those are the optical axes; they are not. Measured on skybeta-03 (drone flat, 27 deg
    holder): gravity along +x and -z, none on y, so up ~ (cos p, 0, -sin p) in IMU axes."""
    x, _y, z = accel_imu
    return math.degrees(math.atan2(-z, x))


def imu_pitch_deg(accel_in_optical):
    """Camera pitch DOWN from a still IMU's specific force expressed in the optical frame
    (x right, y down, z forward): up = (0, -cos p, -sin p)."""
    x, y, z = accel_in_optical
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    return math.degrees(math.atan2(-z / n, -y / n))
