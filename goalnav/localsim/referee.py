"""Independent run judge: true pose, the drawn checkpoint lines and the rulebook.

Uses simulator truth, never the drone's estimate. Checkpoints use the saved
drawing's lines with the whole circular footprint (literal rule 7.6) and,
separately, the drone centre only, on lines widened by HIT_SLACK: checkpoints
are judged by people watching, by eye, and the whole drone cannot cross CP1
(0.46 m from the net) at all. CP4 is credited through Gate B or the window, whichever the
run crossed; the window adds its 5-point bonus on a finished run.

CP5 is judged where the aircraft rests on the floor once disarmed, with the
whole footprint inside the pad's outer ring; the pad disarm and the first
ground contact times are reported.

The return (rule 7.10) is judged after the pad disarm: take off again, land
and disarm within 2 ft of the launch mark inside the 3-minute limit, without
crashing. It multiplies a finished run's points by 1.25. The run time is
end to end, as the judge's stopwatch takes it: from leaving the launch mark
to the return's disarm on it (or the pad disarm without a return).
"""
from dataclasses import replace
import math

from drone_ref.checkpoint_contract import load_event_reference
from drone_ref.checkpoints import CheckpointMonitor
from drone_ref.scoring import score_run

ROUTES = ('gate_b', 'window')
JUDGES = {'strict': None, 'centre': 1e-3}
# Checkpoints are judged by people watching, by eye: the centre judge's lines
# are this much wider each end and this far back toward the approach (the
# strict judge keeps the drawing).
HIT_SLACK = 0.15
RETURN_TOLERANCE = 0.6096      # 2 ft
TIME_LIMIT = 180.


def _widened(line, slack=HIT_SLACK):
    """A checkpoint line ``slack`` longer each end and ``slack`` back toward
    the approach side."""
    a, b = (tuple(map(float, p)) for p in (line.start_xy, line.end_xy))
    n = line.forward_normal_xy
    length = math.dist(a, b)
    t = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
    move = lambda p, sign: (p[0] + sign * t[0] * slack - n[0] * slack, p[1] + sign * t[1] * slack - n[1] * slack)
    return replace(line, start_xy=move(a, -1), end_xy=move(b, 1))


class Referee:
    def __init__(self, course, footprint_radius, crash_speed=1.0, crash_drop_speed=2.0, touchdown=True):
        self.radius = footprint_radius
        self.use_lines({route: load_event_reference(course, route=route).checkpoints for route in ROUTES})
        self.touchdown = touchdown       # CP5 on resting on the pad armed (the team's reading), not only disarmed
        self.crash_speed = crash_speed
        self.pad = course['landing']
        self.left_launch = None
        self.touches = []            # (time since leaving launch, speed, absolute time)
        self.in_contact = False
        self.crashed = None
        self.finished = None
        self.ground_contact = None
        self.crash_drop_speed = crash_drop_speed
        self.launch = (course['launch']['x'], course['launch']['y'])
        self.return_left = None
        self.return_done = None
        self.return_failed = None
        self.return_distance = None
        self.was_armed = False
        self.disarmed = None         # Time since leaving launch of the pad disarm.
        self.contact = None          # (time since leaving launch, xy) of the first floor contact.
        self.rest = None             # xy at rest on the floor, disarmed.

    def use_lines(self, lines):
        """Judge these checkpoint lines ({route: [Checkpoint]}): the strict
        judge as drawn, the centre judge widened (HIT_SLACK)."""
        self.lines = lines
        judged = {'strict': lines, 'centre': {route: [_widened(c) for c in cps] for route, cps in lines.items()}}
        self.monitors = {(judge, route): CheckpointMonitor(judged[judge][route], radius or self.radius)
                         for judge, radius in JUDGES.items() for route in lines}
        self.crossings = {key: {} for key in self.monitors}

    def all_lines(self):
        seen = {}
        for lines in self.lines.values():
            for line in lines:
                seen[line.name] = line
        return list(seen.values())

    def impact(self, now, vertical_speed):
        """The aircraft hit the floor unpowered (a disarm in the air)."""
        if self.left_launch is None or vertical_speed <= self.crash_drop_speed:
            return
        reason = (now - self.left_launch, f'dropped onto the floor at {vertical_speed:.2f} m/s')
        if self.finished is None:
            self.crashed = self.crashed or reason
        elif self.return_left is not None and self.return_done is None:
            self.return_failed = self.return_failed or reason[1]

    def update(self, now, position, velocity, contacts, landed, armed):
        """``landed``: the body truly rests on the floor (not PX4's land detector)."""
        xy = (float(position[0]), float(position[1]))
        if self.left_launch is None and not landed:
            self.left_launch = now
        if self.finished is not None and not self.crashed:
            self._update_return(now, xy, velocity, contacts, landed, armed)
            return
        if self.left_launch is None or self.crashed or self.finished:
            return
        elapsed = now - self.left_launch
        if self.was_armed and not armed and self.disarmed is None:
            self.disarmed = elapsed
        self.was_armed = armed
        for key, monitor in self.monitors.items():
            for crossed in monitor.update(xy).crossed:
                self.crossings[key].setdefault(crossed, elapsed)
        if contacts and not self.in_contact:
            speed = math.hypot(*velocity[:2])
            self.touches.append((elapsed, speed, now))
            if speed > self.crash_speed:
                self.crashed = (elapsed, f'hit an obstacle at {speed:.2f} m/s')
        self.in_contact = bool(contacts)
        if landed and self.contact is None:
            self.contact = (elapsed, xy)
        if landed and (not armed or self.touchdown):
            if math.hypot(*velocity[:2]) > 0.02:
                return                  # Still sliding.
            self.rest = xy
            if self._on_pad(xy):
                self.finished = self.disarmed if self.disarmed is not None else elapsed
            elif self.ground_contact is None:
                self.ground_contact = (elapsed, 'came to rest outside the pad ring')
        elif landed and armed and not self._on_pad(xy) and self.ground_contact is None:
            self.ground_contact = (elapsed, 'touched the ground away from the pad')

    def _on_pad(self, xy):
        return math.hypot(xy[0] - self.pad['x'], xy[1] - self.pad['y']) + self.radius <= self.pad['radius_m']

    def _update_return(self, now, xy, velocity, contacts, landed, armed):
        if self.return_done is not None or self.return_failed is not None:
            return
        elapsed = now - self.left_launch
        if self.return_left is None:
            if not landed:
                self.return_left = elapsed
            return
        if elapsed > TIME_LIMIT:
            self.return_failed = 'over the 3-minute limit'
        elif contacts and not self.in_contact and math.hypot(*velocity[:2]) > self.crash_speed:
            self.return_failed = f'hit an obstacle at {math.hypot(*velocity[:2]):.2f} m/s'
        elif landed and not armed:
            self.return_distance = math.dist(xy, self.launch)
            if self.return_distance <= RETURN_TOLERANCE:
                self.return_done = elapsed
            else:
                self.return_failed = f'landed {self.return_distance:.2f} m from the launch mark'
        self.in_contact = bool(contacts)

    def _judge(self, judge, route, fastest_finish):
        crossed = self.crossings[(judge, route)]
        times = []
        for line in self.lines[route]:
            if line.name not in crossed:
                break
            times.append(crossed[line.name])
        finished = len(times) == len(self.lines[route]) and self.finished is not None
        checkpoints = len(times) + (1 if finished else 0)
        returned = finished and self.return_done is not None
        elapsed = (self.return_done if returned else self.finished) if finished else (times[-1] if times else 0.)
        touches = sum(1 for t, _, _ in self.touches if t <= elapsed)
        score = None
        if checkpoints:
            adjusted = elapsed + 5 * touches
            fastest = min(fastest_finish or adjusted, adjusted)
            score = score_run(checkpoints, elapsed, fastest, touches=touches,
                              window=finished and route == 'window').run
        return {'route': route, 'checkpoints': checkpoints, 'checkpoint_times_s': times,
                'elapsed_s': elapsed, 'touches': touches, 'run_points_if_fastest': score,
                'with_return_bonus': round(score * 1.25, 2) if returned and score is not None else score}

    def summary(self, fastest_finish=None):
        out = {}
        for judge in JUDGES:
            results = [self._judge(judge, route, fastest_finish) for route in ROUTES]
            out[judge] = max(results, key=lambda r: (r['checkpoints'], r['run_points_if_fastest'] or 0))
        out['finish_s'] = self.finished
        out['total_s'] = self.return_done if self.return_done is not None else self.finished
        out['disarm_s'] = self.disarmed
        out['contact_s'] = self.contact[0] if self.contact else None
        out['contact_xy'] = list(self.contact[1]) if self.contact else None
        out['rest_xy'] = list(self.rest) if self.rest else None
        if self.rest:
            out['rest_ring_slack_m'] = self.pad['radius_m'] - self.radius - math.hypot(
                self.rest[0] - self.pad['x'], self.rest[1] - self.pad['y'])
        out['crashed'] = self.crashed
        out['ground_contact'] = self.ground_contact
        out['touch_events'] = [(t, s) for t, s, _ in self.touches]
        out['return'] = {'done_s': self.return_done, 'failed': self.return_failed,
                         'distance_m': self.return_distance, 'left_pad_s': self.return_left}
        return out
