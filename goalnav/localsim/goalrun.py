"""Fly the goal navigator on an irregular, tarp-covered course and judge it.

    python3 -m localsim.goalrun --seed 3                # one course, writes viewer.html
    python3 -m localsim.goalrun --seed 3 --severity 0   # the CAD layout, still tarped and noisy

The course (``localsim.messy_course``) is new for every seed: built like the
real one (plywood stands, one-sided tarps, wire tunnel) and placed off the
CAD, an almost invisible net with a busy room behind it, stereo noise. The
referee's checkpoint lines move with their walls. The drone gets only its
sensors: stereo depth and RGB from the OAK-D (mounted pitched 26.7 degrees
down), PX4 state and the downward rangefinder. It finds the landing target
in the RGB image itself (``drone_ref.goalnav.pad_detect``).
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from localsim import fakeros as m                      # noqa: E402
from localsim import messy_course                      # noqa: E402
from localsim.aero import WallField                    # noqa: E402
from localsim.camrec import CameraRecorder             # noqa: E402
from localsim.messy_rgb import FloorPainter, RGBCamera, render as render_rgb   # noqa: E402
from localsim.messy_world import MessyWorld            # noqa: E402
from localsim.referee import Referee                   # noqa: E402
from localsim.run import camera_messages, stamp        # noqa: E402
from localsim.vehicle import Vehicle, VehicleConfig    # noqa: E402
from localsim.world import Camera                      # noqa: E402

PHYSICS_DT = 0.005
DEPTH_TOPIC, INFO_TOPIC = '/oak/stereo/image_raw', '/oak/stereo/camera_info'
RGB_TOPIC, RGB_INFO_TOPIC = '/oak/rgb/image_raw', '/oak/rgb/camera_info'
CAMERA_PITCH_DEG = 26.7      # cad/modded-cam.stl: the camera face points 26.7 degrees down


class Recorder:
    """What the viewer shows: truth, the drone's belief, its map and plan."""

    def __init__(self, vehicle):
        self.origin = vehicle.frame_origin.copy()
        self.yaw = vehicle.frame_yaw
        self.frames, self.maps, self.clouds = [], [], []
        self.previous = set()
        self.cams, self.detector = None, None     # Camera frames (localsim.camrec), if kept.

    def to_course(self, odom_xy):
        """Odometry ENU (as PX4's frame starts) -> course XY, no drift."""
        x, y = np.asarray(odom_xy, float)[..., 0], np.asarray(odom_xy, float)[..., 1]
        s, c = math.sin(self.yaw), math.cos(self.yaw)
        return np.stack([self.origin[0] + x * s + y * c, self.origin[1] - x * c + y * s], axis=-1)

    def frame(self, t, vehicle, mission, contacts):
        out = {'t': round(t, 2), 'p': [round(float(v), 3) for v in vehicle.p], 'yaw': round(float(vehicle.yaw), 3),
               'speed': round(float(np.hypot(*vehicle.v[:2])), 2), 'state': mission.state, 'touch': int(contacts)}
        tel = mission.telemetry
        if tel is not None:
            est = self.to_course(tel.position[:2])
            out['est'] = [round(float(est[0]), 3), round(float(est[1]), 3),
                          round(float(tel.position[2] - (mission.floor.floor_z if mission.floor else 0) - 0.0), 3)]
        route = mission.route
        if route is not None:
            out['goal'] = route.current.id if route.current else None
            out['goals'] = [[g.id, *[round(float(v), 3) for v in self.to_course(g.estimate)],
                             int(g.detected is not None), int(g.passed is not None)] for g in route.goals]
            out['heading_deg'] = round(math.degrees(route.heading_correction), 2)
        if mission.path is not None:
            out['path'] = [[round(float(a), 3), round(float(b), 3)] for a, b in self.to_course(np.array(mission.path))]
        fence = getattr(mission, 'fence', None)
        if fence is not None:
            c, s = math.cos(fence.yaw), math.sin(fence.yaw)
            corners = [np.asarray(fence.centre) + np.array([c * u - s * v, s * u + c * v])
                       for u, v in ((-fence.half[0], -fence.half[1]), (fence.half[0], -fence.half[1]),
                                    (fence.half[0], fence.half[1]), (-fence.half[0], fence.half[1]))]
            out['fence'] = np.round(self.to_course(np.array(corners)), 3).tolist()
            out['net_shift'] = {k: round(v, 3) for k, v in getattr(mission, 'net_shift', {}).items()}
        w = mission.window
        if w.goal_id is not None and route is not None and route.current is not None and route.current.id == w.goal_id:
            out['window'] = [w.phase, round(float(w.s), 3), round(float(getattr(w, 'integral', 0.)), 3), int(getattr(w, 'backing', False))]
        if route is not None and route.current is not None and mission.floor is not None:
            out['gh'] = round(float(mission._goal_height(route.current)), 3)
        out['limit'] = round(mission.speed, 2)
        out['free'] = round(mission.free_ahead, 2)
        if mission.floor is not None and mission.floor.last:
            out['range'] = [mission.floor.last[0], None if mission.floor.last[1] is None
                            else round(float(mission.floor.last[1]), 3)]
        self.frames.append(out)

    def aim(self, mission, true_xy):
        """Where the drone will really come down if it lands on its aim now,
        in course XY: its aim, less its current odometry drift (None before
        the pad is in sight)."""
        if mission.state == 'PAD_DESCEND' and getattr(mission, 'pad_xy', None) is not None:
            aim = mission.pad_xy
        else:
            route = mission.route
            pad = next((g for g in route.goals if g.kind == 'pad'), None) if route is not None else None
            if pad is None or pad.detected is None or mission.telemetry is None:
                return None
            aim = getattr(pad, 'landing', pad.detected)
        drift = self.to_course(mission.telemetry.position[:2]) - np.asarray(true_xy[:2])
        return self.to_course(aim) - drift

    def camera_frame(self, t, rgb_camera, colour, depth_camera, depth, pose, mission, pad):
        detection = self.detector.last if self.detector is not None else None
        marks = {'H': (pad['x'], pad['y'], 0.)}
        aim = self.aim(mission, pose[0])
        if aim is not None:
            marks['aim'] = (float(aim[0]), float(aim[1]), 0.)
        self.cams.add(t, rgb_camera, colour, depth_camera, depth, pose, detection, marks)

    def map_snapshot(self, t, mission):
        live = mission.map
        if live is None:
            return
        voxels = live.snapshot()
        keys = set((voxels[:, 0] * live.n + voxels[:, 1]) * live.n + voxels[:, 2]) if len(voxels) else set()
        keys = {int(k) for k in keys}
        added, removed = keys - self.previous, self.previous - keys
        self.previous = keys
        self.maps.append({'t': round(t, 2), 'add': sorted(added), 'del': sorted(removed)})

    def map_geometry(self, mission):
        live = mission.map
        if live is None:
            return None
        corner = self.to_course(live.origin[None])[0]
        return {'n': live.n, 'res': live.res, 'band': live.band, 'floor_z': live.floor_z - (live.floor_z),
                'origin_course': [float(v) for v in corner], 'axis_x': self.to_course(np.array([[1., 0.]]))[0].tolist(),
                'zero': self.to_course(np.array([[0., 0.]]))[0].tolist()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--severity', type=float, default=1.0)
    parser.add_argument('--duration', type=float, default=150.)
    parser.add_argument('--camera-hz', type=float, default=15.)
    parser.add_argument('--camera-latency', type=float, default=0.04)
    parser.add_argument('--set', action='append', default=[], metavar='PARAM=VALUE')
    parser.add_argument('--vehicle', action='append', default=[], metavar='FIELD=VALUE')
    parser.add_argument('--out', default=None)
    parser.add_argument('--no-viewer', action='store_true')
    parser.add_argument('--no-camera', action='store_true', help='do not keep the RGB and depth frames')
    parser.add_argument('--profile', default='safe', help='risk profile in config/goalnav_profiles.yaml')
    parser.add_argument('--tune', action='append', default=[], metavar='KEY=VALUE',
                        help='override one profile setting, e.g. --tune max_speed=0.7 --tune cp4_route=window')
    args = parser.parse_args(argv)
    out = Path(args.out or ROOT / f'localsim/out/goalnav/seed{args.seed}')
    out.mkdir(parents=True, exist_ok=True)
    wall = time.time()
    course = messy_course.generate(args.seed, args.severity)
    obj, meta_path = messy_course.save(course, out / 'course')
    meta = json.loads(meta_path.read_text())
    world = MessyWorld(obj, meta, seed=args.seed, cache_dir=ROOT / 'localsim/.cache')
    params = {'drone_ref_goalnav': {'depth_topic': DEPTH_TOPIC, 'camera_info_topic': INFO_TOPIC,
                                    'rgb_topic': RGB_TOPIC, 'rgb_info_topic': RGB_INFO_TOPIC,
                                    'camera_pitch_deg': CAMERA_PITCH_DEG,
                                    'goals_file': str(ROOT / 'config/goals.yaml'),
                                    'profile': args.profile,
                                    'tuning': yaml.safe_dump({k: yaml.safe_load(v) for k, v in
                                                              (item.split('=', 1) for item in args.tune)}) if args.tune else ''}}
    for item in args.set:
        key, value = item.split('=', 1)
        params['drone_ref_goalnav'][key] = yaml.safe_load(value)
    bus = m.Bus(params)
    bus.ns = 5_000_000_000
    # The drone's OAK-D: fx 398.7 px at 640x400 (77.5 deg), minimum range 0.31 m.
    camera = Camera(pitch_deg=CAMERA_PITCH_DEG, hfov_deg=77.5, min_depth=0.31)
    cad_course = json.loads((ROOT / 'config/course_v3.json').read_text())
    referee = Referee(cad_course, messy_course.BODY_RADIUS)
    referee.lines = {route: messy_course.checkpoints(meta, lines) for route, lines in referee.lines.items()}
    from drone_ref.checkpoints import CheckpointMonitor
    from localsim.referee import JUDGES
    referee.monitors = {(judge, route): CheckpointMonitor(referee.lines[route], radius or referee.radius)
                        for judge, radius in JUDGES.items() for route in referee.lines}
    referee.pad = meta['pad']
    referee._on_pad = lambda xy: on_mat(meta['pad'], xy)      # No ring round the target: rest on the mat.
    referee.launch = tuple(meta['launch']['mark'])
    rng = np.random.default_rng(args.seed + 17)
    defaults = {'drift_m_per_sqrt_s': 0.008, 'yaw_drift_rad_per_sqrt_s': 0.0015}
    defaults.update({k: float(v) for k, v in (item.split('=', 1) for item in args.vehicle)})
    with m.installed(bus, ROOT):
        from drone_ref import goalnav_node
        node = goalnav_node.GoalNavNode()
        mission = node.mission
        launch = meta['launch']
        walls = WallField.try_load(world, obj, ROOT / 'localsim/.cache')
        vehicle = Vehicle(bus, (launch['xy'][0], launch['xy'][1], 0.), math.radians(launch['yaw_deg']),
                          VehicleConfig(**defaults), seed=args.seed, world=world, walls=walls)
        recorder = Recorder(vehicle)
        if not args.no_camera:
            recorder.cams, recorder.detector = CameraRecorder(out), node.pad_detector
        result = fly(args, bus, world, camera, vehicle, mission, referee, recorder, meta, rng)
    result['wall_time_s'] = round(time.time() - wall, 1)
    result['profile'] = {'name': args.profile, 'tune': args.tune}
    result['camera_pitch_deg'] = CAMERA_PITCH_DEG
    result['course'] = {'seed': args.seed, 'severity': args.severity, 'attempt': meta['attempt'],
                        'launch': meta['launch'], 'tunnel_cut': [meta['tunnel']['cut'], meta['tunnel']['cut_end']]}
    (out / 'report.json').write_text(json.dumps(result, indent=1, default=str))
    trace = {'frames': recorder.frames, 'maps': recorder.maps, 'clouds': recorder.clouds,
             'map': recorder.map_geometry(mission), 'lines': meta['lines'], 'pad': meta['pad'],
             'nets': meta['nets'], 'launch': meta['launch'], 'result': result}
    if recorder.cams is not None:
        trace['cams'] = recorder.cams.summary(RGBCamera(pitch_deg=CAMERA_PITCH_DEG), camera)
    (out / 'trace.json').write_text(json.dumps(trace, separators=(',', ':'), default=float))
    if not args.no_viewer:
        from localsim import live_index, viewer3d
        viewer3d.build(out)
        live_index.update(out.parent)          # The live page (localsim/live_index.py) picks it up.
    summary = {k: result[k] for k in ('final', 'finish_s', 'total_s', 'checkpoints', 'route', 'touches', 'crashed')}
    print(json.dumps(summary, default=str))
    print('Viewer:', out / 'viewer.html')
    return 0 if result['finish_s'] else 1


def fly(args, bus, world, camera, vehicle, mission, referee, recorder, meta, rng):
    begin = bus.ns
    start_at, end = begin + int(3e9), begin + int(args.duration * 1e9)
    next_fast = next_slow = next_frame = next_trace = next_map = next_range = begin
    frame_period = int(1e9 / args.camera_hz)
    queue, start, settled = [], None, None
    rgb_camera, painter, frames = RGBCamera(pitch_deg=CAMERA_PITCH_DEG), FloorPainter(meta), 0
    next_cloud = begin
    nets = meta['nets']
    info = {}
    while bus.ns < end:
        vehicle.step(PHYSICS_DT)
        bus.ns += int(PHYSICS_DT * 1e9)
        world.now = bus.seconds
        if bus.ns >= next_fast:
            vehicle.publish_fast()
            next_fast += 20_000_000
        if bus.ns >= next_slow:
            vehicle.publish_slow()
            next_slow += 100_000_000
        if bus.ns >= next_range:
            distance = world.range_down(vehicle.p, vehicle.rotation())
            bus.publish('/fmu/out/distance_sensor', m.DistanceSensor(
                timestamp=bus.ns // 1000, current_distance=float(distance or 0.),
                signal_quality=100 if distance is not None else 0))
            next_range += 20_000_000
        if bus.ns >= next_frame:
            depth = world.render_depth(camera, vehicle.p, vehicle.rotation())
            frames += 1
            colour = render_rgb(world, rgb_camera, painter, vehicle.p, vehicle.rotation(), rng) \
                if frames % 3 == 0 else None                  # RGB at 5 Hz
            queue.append((bus.ns + int(args.camera_latency * 1e9), bus.ns, depth, colour,
                          (vehicle.p.copy(), vehicle.rotation())))
            next_frame += frame_period
            if bus.ns >= next_cloud and bus.ns >= start_at:
                recorder.clouds.append(_cloud(bus.seconds - begin / 1e9, camera, depth, vehicle))
                next_cloud = bus.ns + 200_000_000
        shown = None
        while queue and queue[0][0] <= bus.ns:
            _, captured, depth, colour, pose = queue.pop(0)
            _, depth_msg, info_msg = camera_messages(camera, depth, captured, 'oak_optical')
            bus.publish(DEPTH_TOPIC, depth_msg)
            bus.publish(INFO_TOPIC, info_msg)
            if colour is not None:
                for topic, message in rgb_messages(rgb_camera, colour, captured):
                    bus.publish(topic, message)
                shown = ((captured - begin) / 1e9, colour, depth, pose)
        if shown is not None and recorder.detector is not None:
            recorder.detector.last = None              # Set again only if the node looks at this frame.
        bus.drain()
        if shown is not None and recorder.cams is not None and bus.ns >= start_at:
            recorder.camera_frame(shown[0], rgb_camera, shown[1], camera, shown[2], shown[3], mission, meta['pad'])
        bus.run_timers()
        if start is None and bus.ns >= start_at:
            start = bus.call('/goalnav/start')
            info['start'] = [start.success, start.message]
            if not start.success:
                break
        while vehicle.impacts:
            when, vertical, _ = vehicle.impacts.pop(0)
            referee.impact(when, vertical)
        contacts = 0
        if not vehicle.landed:
            contacts = drone_contacts(world, vehicle.p, referee.radius)
            r = referee.radius
            x, y = vehicle.p[:2]
            if x - r < -nets['west'] or x + r > nets['east'] or y - r < -nets['south'] or y + r > nets['north']:
                contacts += 1
                info.setdefault('net_touches', []).append(round(bus.seconds - begin / 1e9, 2))
        referee.update(bus.seconds, vehicle.p, vehicle.v, contacts, vehicle.on_ground, vehicle.armed)
        if contacts:
            info['contact_s'] = info.get('contact_s', 0.) + PHYSICS_DT
            deep = contacts > 40          # Voxels inside the body: well into the wall, not a graze.
            if (deep or info['contact_s'] > 1.0) and not referee.crashed and referee.finished is None:
                referee.crashed = (bus.seconds - referee.left_launch,
                                   'pushed deep into a wall' if deep else 'in contact with a wall for 1 s')
        else:
            info['contact_s'] = 0.
        t = bus.seconds - begin / 1e9
        if bus.ns >= next_trace:
            recorder.frame(t, vehicle, mission, contacts)
            next_trace += 100_000_000
        if bus.ns >= next_map:
            recorder.map_snapshot(t, mission)
            next_map += 500_000_000
        if mission.state == 'ON_PAD' and 'pad_landing' not in info:
            info['pad_landing'] = pad_landing(vehicle.p, recorder.aim(mission, vehicle.p), meta['pad'])
        if referee.crashed:
            break
        if mission.state in ('DONE', 'ABORTED') and vehicle.landed:
            settled = settled or bus.ns
            if bus.ns - settled > 3e9:
                break
    recorder.map_snapshot(bus.seconds - begin / 1e9, mission)
    summary = referee.summary()
    centre = summary['centre']
    return {'final': [mission.state, mission.status.reason], 'start': info.get('start'),
            'finish_s': summary['finish_s'], 'total_s': summary['total_s'], 'route': centre['route'],
            'checkpoints': centre['checkpoints'], 'checkpoint_times_s': centre['checkpoint_times_s'],
            'touches': len(summary['touch_events']), 'touch_events': summary['touch_events'][:20],
            'strict': summary['strict'], 'crashed': summary['crashed'],
            'ground_contact': summary['ground_contact'], 'return': summary['return'],
            'net_touches': info.get('net_touches', [])[:20], 'events': mission.status.events,
            'pad_landing': info.get('pad_landing'),
            'vehicle_events': vehicle.events[-30:], 'rest_xy': summary['rest_xy'],
            'floor_tracker': {'accepted': mission.floor.accepted if mission.floor else 0,
                              'rejected': mission.floor.rejected if mission.floor else 0},
            'errors': [e for e in bus.log if e[2] in ('error', 'warning')][-10:]}


def drone_contacts(world, p, radius, core_radius=0.13, arm_radius=0.20):
    """Contact voxels for the Sky Beta's shape (its CAD; body origin at the
    Pixhawk, 7 cm over the feet, rigidbody.body_height), in three layers:
    plates, Jetson and battery (the core) from the feet to 5 cm over the
    origin; arms and motors out to ``arm_radius`` 3.8-7.4 cm over the feet;
    the props, full ``radius``, 7.4-9.2 cm over the feet (rotor tops measured)."""
    core = np.array([p[0], p[1], p[2] - 0.01])
    arms = np.array([p[0], p[1], p[2] - 0.014])
    props = np.array([p[0], p[1], p[2] + 0.013])
    return (world.contacts(core, core_radius, 0.06) + world.contacts(arms, arm_radius, 0.018)
            + world.contacts(props, radius, 0.009))


def pad_landing(p, aim, pad):
    """Touchdown against the H's centre: where the drone came down, and where it aimed."""
    centre = np.array([pad['x'], pad['y']])
    out = {'error_m': round(float(np.linalg.norm(np.asarray(p[:2]) - centre)), 3)}
    if aim is not None:
        out['aim_error_m'] = round(float(np.linalg.norm(np.asarray(aim) - centre)), 3)
    return out


def on_mat(pad, xy):
    """CP5 on the real target (no ring): the drone's centre rests on the mat."""
    yaw = math.radians(pad.get('yaw_deg', 0.))
    dx, dy = xy[0] - pad['x'], xy[1] - pad['y']
    u, v = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
    half = pad.get('mat_m', 0.85) / 2
    return abs(u) <= half and abs(v) <= half


def rgb_messages(camera, image, ns):
    header = m.Header(stamp=stamp(ns), frame_id='oak_rgb_optical')
    msg = m.Image(header=header, height=camera.height, width=camera.width, encoding='rgb8',
                  step=camera.width * 3, data=m.array('B', image.tobytes()))
    k = np.array([camera.fx, 0, camera.width / 2, 0, camera.fx, camera.height / 2, 0, 0, 1.])
    info = m.CameraInfo(header=header, height=camera.height, width=camera.width, k=k)
    return [(RGB_TOPIC, msg), (RGB_INFO_TOPIC, info)]


def _cloud(t, camera, depth_mm, vehicle, keep=320):
    """A thin sample of what the camera returned, in course coordinates."""
    s = camera.stride
    block = depth_mm[s // 2::s, s // 2::s].astype(float) / 1000.
    v, u = np.nonzero(block > 0)
    if not len(v):
        return {'t': round(t, 2), 'pts': []}
    pick = np.linspace(0, len(v) - 1, min(keep, len(v))).astype(int)
    v, u = v[pick], u[pick]
    z = block[v, u]
    px, py = u * s + s / 2, v * s + s / 2
    optical = np.column_stack([(px - camera.cx) * z / camera.fx, (py - camera.cy) * z / camera.fx, z])
    body = optical @ camera.body_from_optical().T + np.asarray(camera.offset)
    world = body @ vehicle.rotation().T + vehicle.p
    return {'t': round(t, 2), 'pts': np.round(world, 2).reshape(-1).tolist()}


if __name__ == '__main__':
    raise SystemExit(main())
