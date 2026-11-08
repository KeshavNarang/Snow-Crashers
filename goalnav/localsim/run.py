"""Fly our real ROS nodes in the local simulator and judge the run.

    python3 -m localsim.run                       # lab profile, nominal
    python3 -m localsim.run --set drone_ref_mission.max_speed=0.8
    python3 -m localsim.run --dark-tunnel --seed 3

Needs Python 3.10+, NumPy and PyYAML. Writes a JSON report and an SVG map.
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

from localsim import fakeros as m                         # noqa: E402
from localsim.aero import WallField                       # noqa: E402
from localsim.referee import Referee                      # noqa: E402
from localsim.vehicle import Vehicle, VehicleConfig       # noqa: E402
from localsim.world import Camera, World                  # noqa: E402
from localsim.plot import write_svg                       # noqa: E402
from localsim import replay                                # noqa: E402

PHYSICS_DT = 0.005


WORLDS = {
    # OAK-D S2 at 640x400: on-device stereo comfortably runs 60 fps; USB + driver
    # latency about 30 ms. The organizers' lab preset renders 15 Hz.
    'hardware': {'camera_hz': 60., 'camera_latency': 0.03},
    'lab': {'camera_hz': 15., 'camera_latency': 0.04},
}


def load_params(paths, overrides):
    params = {}
    for path in paths:          # Later files override earlier ones, like ROS params files.
        for node, section in yaml.safe_load(Path(path).read_text()).items():
            params.setdefault(node, {}).update(section['ros__parameters'])
    for item in overrides:
        key, value = item.split('=', 1)
        node, name = key.split('.', 1)
        params.setdefault(node, {})[name] = yaml.safe_load(value)
    return params


def stamp(ns):
    return m.Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def camera_messages(camera, depth_mm, ns, frame):
    header = m.Header(stamp=stamp(ns), frame_id=frame)
    mono = ((depth_mm // 37) % 200 + 30).astype(np.uint8)
    image = m.Image(header=header, height=camera.height, width=camera.width, encoding='mono8',
                    step=camera.width, data=m.array('B', mono.tobytes()))
    depth = m.Image(header=header, height=camera.height, width=camera.width, encoding='16UC1',
                    step=camera.width * 2, data=m.array('B', depth_mm.tobytes()))
    k = np.array([camera.fx, 0, camera.cx, 0, camera.fx, camera.cy, 0, 0, 1.])
    p = np.array([camera.fx, 0, camera.cx, 0, 0, camera.fx, camera.cy, 0, 0, 0, 1., 0])
    info = m.CameraInfo(header=header, height=camera.height, width=camera.width, k=k, p=p)
    return image, depth, info


def camera_transform(camera):
    # base_link <- optical (x right, y down, z forward): quaternion of [[0,0,1],[-1,0,0],[0,-1,0]].
    rotation = m.Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5)
    translation = m.Vector3(x=camera.offset[0], y=camera.offset[1], z=camera.offset[2])
    return type('TransformStamped', (), {'transform': type('T', (), {
        'rotation': rotation, 'translation': translation})()})()


class Simulation:
    def __init__(self, args):
        self.args = args
        self.params = load_params(args.params, args.set)
        self.course = json.loads((ROOT / 'config/course_v3.json').read_text())
        self.camera = Camera()
        self.world = World(ROOT / 'assets/course-v3.obj', cache_dir=ROOT / 'localsim/.cache',
                           dark_tunnel=args.dark_tunnel)
        self.bus = m.Bus(self.params)
        self.bus.ns = 5_000_000_000
        mission = self.params['drone_ref_mission']
        self.referee = Referee(self.course, mission.get('body_radius', .28))
        self.trace, self.status, self.vision_status = [], None, None
        self.start_response = None
        self.camera_queue = []

    def run(self):
        args, bus = self.args, self.bus
        vision = self.params['drone_ref_vision']
        with m.installed(bus, ROOT):
            from drone_ref import mission_node, vision_node
            bus.tf[(vision.get('base_frame', 'base_link'), vision['optical_frame'])] = camera_transform(self.camera)
            mission_node.main(args=[])
            vision_node.main(args=[])
            self.node = bus.nodes[0]
            launch = self.course['launch']
            overrides = {k: float(v) for k, v in (item.split('=', 1) for item in args.vehicle)}
            walls = WallField.try_load(self.world, ROOT / 'assets/course-v3.obj', ROOT / 'localsim/.cache')
            dx, dy = args.launch_offset
            vehicle = self._vehicle(bus, (launch['x'] + dx, launch['y'] + dy, 0.), math.radians(args.launch_yaw_deg),
                                    overrides, walls)
            bus.subscribe('/drone_ref/mission_status', lambda msg: setattr(self, 'status', json.loads(msg.data)))
            bus.subscribe('/drone_ref/vision_status', lambda msg: setattr(self, 'vision_status', json.loads(msg.data)))
            try:
                self._loop(vehicle)
            finally:
                getattr(vehicle, 'close', lambda: None)()
        return self.report(vehicle)

    def _vehicle(self, bus, position, yaw, overrides, walls):
        args = self.args
        if not args.px4:
            return Vehicle(bus, position, yaw, VehicleConfig(**overrides), seed=args.seed,
                           world=self.world, walls=walls)
        from localsim.px4.vehicle import PX4Config, PX4Vehicle
        Path(args.out).mkdir(parents=True, exist_ok=True)
        return PX4Vehicle(bus, position, yaw, PX4Config(**overrides), seed=args.seed, world=self.world,
                          walls=walls, instance=args.px4_instance, param_files=args.px4_params, out_dir=args.out)

    def _loop(self, vehicle):
        args, bus = self.args, self.bus
        begin = bus.ns
        start_at = begin + int(args.start_after * 1e9)
        end = begin + int(args.duration * 1e9)
        next_fast = next_slow = next_frame = next_trace = begin
        settled_since = None
        frame_period = int(1e9 / args.camera_hz)
        dt = getattr(vehicle, 'dt', PHYSICS_DT)
        while bus.ns < end:
            vehicle.step(dt)
            bus.ns += int(dt * 1e9)
            if bus.ns >= next_fast:
                vehicle.publish_fast()
                next_fast += 20_000_000
            if bus.ns >= next_slow:
                vehicle.publish_slow()
                next_slow += 100_000_000
            if bus.ns >= next_frame:
                # Exact wall depth on the pad, where the launch alignment uses it.
                depth = self.world.render_depth(self.camera, vehicle.p, vehicle.rotation(),
                                                exact=self.start_response is None)
                self.camera_queue.append((bus.ns + int(args.camera_latency * 1e9), bus.ns, depth))
                next_frame += frame_period
            while self.camera_queue and self.camera_queue[0][0] <= bus.ns:
                _, captured, depth = self.camera_queue.pop(0)
                image, depth_msg, info = camera_messages(self.camera, depth, captured,
                                                         self.params['drone_ref_vision']['optical_frame'])
                for topic, message in ((self.params['drone_ref_vision']['image_topic'], image),
                                       (self.params['drone_ref_vision']['depth_topic'], depth_msg),
                                       (self.params['drone_ref_vision']['camera_info_topic'], info)):
                    bus.publish(topic, message)
            bus.drain()
            bus.run_timers()
            if self.start_response is None and bus.ns >= start_at:
                self.start_response = bus.call('/drone_ref/start')
                if not self.start_response.success:
                    break
            while vehicle.impacts:
                when, vertical, _ = vehicle.impacts.pop(0)
                self.referee.impact(when, vertical)
            contacts = self.world.contacts(vehicle.p, self.referee.radius, .16) if not vehicle.landed else 0
            self.referee.update(bus.seconds, vehicle.p, vehicle.v, contacts, vehicle.on_ground, vehicle.armed)
            if bus.ns >= next_trace:
                state = self.node.mission.state if self.node.mission else 'IDLE'
                reason = self.node.mission.reason if self.node.mission else ''
                self.trace.append((round(bus.seconds - begin / 1e9, 3), *map(float, vehicle.p),
                                   float(np.hypot(*vehicle.v[:2])), float(vehicle.yaw), state, int(contacts),
                                   float(vehicle.v[0]), float(vehicle.v[1]), reason))
                next_trace += 50_000_000
            if self.referee.crashed:
                break
            mission = self.node.mission
            if mission and mission.state in mission.TERMINAL and vehicle.landed:
                settled_since = settled_since or bus.ns
                if bus.ns - settled_since > 3e9:
                    break

    def report(self, vehicle):
        mission = self.node.mission
        planned = [(x, y) for x, y, _ in self.node.map_route]
        result = {
            'start': {'success': bool(self.start_response and self.start_response.success),
                      'message': self.start_response.message if self.start_response else None},
            'final_state': mission.state if mission else 'IDLE',
            'final_reason': mission.reason if mission else (self.status or {}).get('reason'),
            'route_issue': self.node.route_issue,
            'planned_samples': len(planned),
            'referee': self.referee.summary(self.args.fastest),
            'readiness': (self.status or {}).get('readiness'),
            'placement': {'yaw_deg': self.args.launch_yaw_deg - 90., 'x': self.args.launch_offset[0],
                          'y': self.args.launch_offset[1]},
            'launch_align': self.node.aligner.status() if self.node.aligner else None,
            'vision_status': self.vision_status,
            'vehicle_events': vehicle.events,
            'errors': [entry for entry in self.bus.log if entry[2] in ('error', 'warning')][-20:],
        }
        return result, planned


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--params', action='append', default=None,
                        help='parameter YAML; repeat to overlay (default: config/sim.yaml)')
    parser.add_argument('--world', choices=sorted(WORLDS), default='hardware',
                        help='camera model: the real OAK-D (hardware) or the lab preset')
    parser.add_argument('--set', action='append', default=[], metavar='NODE.PARAM=VALUE')
    parser.add_argument('--duration', type=float, default=200.)
    parser.add_argument('--start-after', type=float, default=3.)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--vehicle', action='append', default=[], metavar='FIELD=VALUE',
                        help='override a localsim.vehicle.VehicleConfig field, e.g. vel_p=2.5')
    parser.add_argument('--launch-yaw-deg', type=float, default=90., help='true heading on the pad (CAD frame)')
    parser.add_argument('--launch-offset', type=float, nargs=2, default=(0., 0.), metavar=('DX', 'DY'),
                        help='true position on the pad relative to the launch mark (CAD metres)')
    parser.add_argument('--dark-tunnel', action='store_true', help='no depth inside the tunnel')
    parser.add_argument('--camera-hz', type=float, default=None)
    parser.add_argument('--camera-latency', type=float, default=None)
    parser.add_argument('--fastest', type=float, default=None, help='fastest finish to score against')
    parser.add_argument('--out', default=str(ROOT / 'localsim/out'))
    parser.add_argument('--px4', action='store_true',
                        help='fly real PX4 1.16 SITL on the rigid-body model (localsim/px4)')
    parser.add_argument('--px4-params', action='append', default=[], metavar='FILE',
                        help='PX4 parameter file to apply; repeatable')
    parser.add_argument('--px4-instance', type=int, default=0, help='PX4 instance (ports 4560+N, 14540+N)')
    args = parser.parse_args(argv)
    args.params = args.params or [str(ROOT / 'config/sim.yaml')]
    for key, value in WORLDS[args.world].items():
        if getattr(args, key) is None:
            setattr(args, key, value)
    wall = time.time()
    sim = Simulation(args)
    result, planned = sim.run()
    result['wall_time_s'] = round(time.time() - wall, 1)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps(result, indent=2, default=str))
    (out / 'trace.json').write_text(json.dumps(sim.trace))
    write_svg(out / 'run.svg', sim.course, planned, sim.trace, sim.referee, start_offset=5.)
    (out / 'plan.json').write_text(json.dumps(planned))
    (out / 'plan_speeds.json').write_text(json.dumps(list(sim.node.map_speeds or [])))
    (out / 'lines.json').write_text(json.dumps([
        {'label': ('CP%d' % line.checkpoint) + (' ' + line.name if line.checkpoint == 4 else ''),
         'start': list(line.start_xy), 'end': list(line.end_xy)} for line in sim.referee.all_lines()]))
    replay.build(out)
    referee = result['referee']
    print(json.dumps({'start': result['start'], 'final': [result['final_state'], result['final_reason']],
                      'strict': referee['strict'], 'centre': referee['centre'],
                      'finish_s': referee['finish_s'], 'total_s': referee['total_s'],
                      'crashed': referee['crashed'],
                      'return': referee['return'],
                      'launch_align': (result['launch_align'] or {}).get('reason'),
                      'wall_time_s': result['wall_time_s']}, indent=1, default=str))
    print('Report:', out / 'report.json', ' Map:', out / 'run.svg', ' Replay:', out / 'replay.html')
    return 0 if referee['finish_s'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
