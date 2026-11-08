#!/usr/bin/env python3
"""driver_swap: switch the running OAK driver to a depth-enabled configuration, or back, in place.

Run INSIDE the skybeta-stack container (ROS sourced), drone lock held, props off, disarmed:

    python3 driver_swap.py health [--depth]          read-only: are the cameras and cuVSLAM healthy right now?
    python3 driver_swap.py candidate --dry-run       show what would happen, change nothing
    python3 driver_swap.py candidate [--size 640x400] [--align left|right|none] [--preset DEFAULT]
                                     [--extended] [--rect] [--pipeline depth|rgbd]
    python3 driver_swap.py restore                   back to the configuration saved before the first swap

How: only the driver COMPONENT is unloaded from the stack's own /oak_container and the new configuration is loaded
into the same container (load_driver.launch.py). No process exits, so the slam pane, cuVSLAM and the PX4 agent keep
running (if the slam pane died, the stack service would restart the agent, which can wedge the FC link).
The configuration is the running driver's own parameter dump plus a checked overlay (driver_params.py); the
organizers' original is saved once, before the first swap, and is what `restore` and the automatic revert load.
If the cameras or cuVSLAM are not healthy after a swap, it reverts by itself. VIO is interrupted for some seconds
during a swap and cuVSLAM may restart its origin: power-cycle before any flight.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import driver_params as dp

HERE = os.path.dirname(os.path.abspath(__file__))
ORIGINAL = 'original_oak_params.yaml'
CANDIDATE = 'candidate_oak_params.yaml'


def parse_component_list(text, container=None):
    """{container: [(id, node)]} from `ros2 component list` (all containers, or one when `container` is given)."""
    comps, current = {}, container
    for line in text.splitlines():
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) == 1 and parts[0].startswith('/'):
            current = parts[0]
            comps.setdefault(current, [])
        elif len(parts) >= 2 and parts[0].isdigit() and current:
            comps.setdefault(current, []).append((int(parts[0]), parts[1]))
    return comps


def find_component(comps, container, node):
    if container not in comps:
        raise LookupError(f'container {container} not found')
    for cid, name in comps[container]:
        if name == node:
            return cid
    raise LookupError(f'{node} is not loaded in {container}')


class Swapper:
    def __init__(self, runner, state_dir, launch_file, container='/oak_container', node='/oak', settle_s=3):
        self.r, self.state, self.launch = runner, state_dir, launch_file
        self.container, self.node, self.settle = container, node, settle_s

    def _path(self, name):
        return os.path.join(self.state, name)

    def _driver_id(self):
        return find_component(parse_component_list(self.r.run(['ros2', 'component', 'list'])), self.container, self.node)

    def _replace(self, params_path):
        cid = self._driver_id()
        self.r.run(['ros2', 'component', 'unload', self.container, str(cid)])
        self.r.run(['sleep', str(self.settle)])                       # let the device close before reopening it
        self.r.run(['ros2', 'launch', self.launch, f'params_file:={params_path}'])

    def swap(self, mode, overlay, dry_run=False):
        try:
            self._driver_id()
        except LookupError as e:
            return {'ok': False, 'why': f'driver component not found: {e}'}
        orig_path = self._path(ORIGINAL)
        if mode == 'restore':
            if not os.path.exists(orig_path):
                return {'ok': False, 'why': 'no saved original configuration: nothing to restore to'}
            plan = [f'unload {self.node} from {self.container}', f'load {orig_path}']
            if dry_run:
                return {'ok': True, 'plan': plan}
            self._replace(orig_path)
            h = self.r.health(want_depth=False)
            return {'ok': h['ok'], 'plan': plan, 'health': h, 'why': '' if h['ok'] else 'restored driver unhealthy: ' + h['why']}

        current = dp.parse_dump(self.r.run(['ros2', 'param', 'dump', self.node]))
        if os.path.exists(orig_path):
            with open(orig_path) as f:
                original_text = f.read()
        else:
            if str(current.get(dp.PIPELINE_KEY, '')).upper() != 'STEREO':
                return {'ok': False, 'why': f'no saved original, and the running config is not the stock stereo pipeline '
                                            f'({current.get(dp.PIPELINE_KEY)!r}); refusing to record it as the original'}
            original_text = dp.to_param_file(current, node=self.node)
            if not dry_run:
                os.makedirs(self.state, exist_ok=True)
                with open(orig_path, 'w') as f:
                    f.write(original_text)
        original = dp.parse_dump(original_text)
        cand = dp.build_candidate(original, overlay)
        problems = dp.check_candidate(original, cand)
        if problems:
            return {'ok': False, 'why': 'candidate refused', 'problems': problems}
        cand_path = self._path(CANDIDATE)
        plan = [f'unload {self.node} from {self.container}', f'load {cand_path}', 'check health; revert if unhealthy']
        if dry_run:
            return {'ok': True, 'plan': plan, 'candidate': cand}
        os.makedirs(self.state, exist_ok=True)
        with open(cand_path, 'w') as f:
            f.write(dp.to_param_file(cand, node=self.node))
        try:
            self._replace(cand_path)
            h = self.r.health(want_depth=True)
        except Exception as e:                                          # anything after the unload: go back
            h = {'ok': False, 'why': f'error during swap: {e}'}
        if h['ok']:
            return {'ok': True, 'plan': plan, 'health': h}
        try:
            self._replace(orig_path)
            back = self.r.health(want_depth=False)
        except Exception as e:
            back = {'ok': False, 'why': f'error during revert: {e}'}
        why = f'candidate unhealthy ({h["why"]}); reverted to the original'
        if not back['ok']:
            why += f' -- RESTORE ALSO UNHEALTHY ({back["why"]}): MANUAL recovery needed'
        return {'ok': False, 'reverted': True, 'restore_healthy': back['ok'], 'health': h, 'why': why}


class ShellRunner:
    """Real runner: executes ros2 commands in this container and measures health from live topics."""

    def __init__(self, timeout=90):
        self.timeout = timeout

    def run(self, cmd):
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=self.timeout)
        if r.returncode != 0:
            raise RuntimeError(f'{" ".join(cmd)} failed ({r.returncode}): {r.stderr.strip()[-400:]}')
        return r.stdout

    def health(self, want_depth, wait_s=40.0, window_s=8.0):
        from live_health import measure                               # ROS imports only on the drone
        return measure(want_depth=want_depth, wait_s=wait_s, window_s=window_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=('health', 'candidate', 'restore'))
    ap.add_argument('--depth', action='store_true', help='health: also require the depth stream')
    ap.add_argument('--size', default='640x400')
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--align', default='left', choices=('left', 'right', 'none'))
    ap.add_argument('--preset', default='DEFAULT', choices=dp.PRESETS)
    ap.add_argument('--extended', action='store_true')
    ap.add_argument('--no-subpixel', action='store_true')
    ap.add_argument('--rect', action='store_true')
    ap.add_argument('--pipeline', default='depth', choices=('depth', 'rgbd'))
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--state-dir', default='/root/team_ws/depth_state')
    a = ap.parse_args()
    runner = ShellRunner()
    if a.mode == 'health':
        print(json.dumps(runner.health(want_depth=a.depth), indent=1, default=str))
        return 0
    w, h = (int(x) for x in a.size.lower().split('x'))
    overlay = dp.depth_overlay(size=(w, h), fps=a.fps, align=a.align, preset=a.preset, extended=a.extended,
                               subpixel=not a.no_subpixel, rect=a.rect, pipeline=a.pipeline)
    t0 = time.time()
    res = Swapper(runner, a.state_dir, os.path.join(HERE, 'load_driver.launch.py')).swap(a.mode, overlay, a.dry_run)
    res['seconds'] = round(time.time() - t0, 1)
    print(json.dumps(res, indent=1, default=str))
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
