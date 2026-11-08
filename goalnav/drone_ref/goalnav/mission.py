"""The goal-navigation mission: perception, planning and the run's phases.

Independent of ROS (``drone_ref.goalnav_node`` is the adapter). Each depth
frame goes into the live map; at ``plan_hz`` the route's goals are looked
for in the map and a path is planned through the next two; at the control
rate the follower turns the path into PX4 setpoints.

Phases: IDLE -> PREPARE (stream setpoints, Offboard, arm) -> TAKEOFF ->
NAVIGATE (the goal chain) -> PAD_DESCEND -> ON_PAD (touched down, CP5; disarmed
only with ``pad_disarm``) ->
RETURN (one move: climbing off the pad, over the barrier, down toward the mark)
-> RETURN_DESCEND -> DONE.
ABORT sends PX4 Land.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from drone_ref.goalnav.altitude import FloorTracker
from drone_ref.goalnav.control import Follower, heading_change
from drone_ref.goalnav.geometry import yaw_of, wrap
from drone_ref.goalnav.goals import Route, opening_axis, refine, wall_line, wall_offset, wall_rotation
from drone_ref.goalnav.floor_line import LineMap, turn_along
from drone_ref.goalnav.livemap import OCCUPIED as OCCUPIED_LOGODDS, Fence, LiveMap
from drone_ref.goalnav.planner import Planner, clearance
from drone_ref.goalnav.return_spot import ReturnSpot
from drone_ref.goalnav.window import WindowManeuver

SET_MODE, ARM, LAND = 176, 400, 21


@dataclass
class Telemetry:
    t: float
    position: np.ndarray          # ENU, odometry frame
    velocity: np.ndarray
    rotation: np.ndarray          # body FLU -> ENU
    armed: bool
    offboard: bool
    landed: bool
    preflight_ok: bool = True
    valid: bool = True


@dataclass
class MissionConfig:
    cruise_height: float = 0.75     # body origin above the floor
    body_height: float = 0.12       # feet to the battery's top (Sky Beta CAD 116 mm; rotor tops 92 mm)
    vertical_margin: float = 0.10   # below the feet: boulders count as obstacles
    landed_base_height: float = 0.07  # body origin over the feet (resting on the floor)
    return_height: float = 2.1
    return_speed: float = 1.2
    return_brake: float = 0.6       # m/s^2 stopping onto the return mark
    return_slope: float = 1.2       # the return's climb and descent, metres up per metre along
    return_margin: float = 0.15     # feet over anything mapped near the return line
    return_reach: float = 0.45      # ... within this of it (rotor tips 0.29 m, some slack)
    return_unseen_top: float = 1.85  # unseen cells on the way back: as tall as a wall
    return_yaw_deg: float = 45.     # CAD heading while returning when goals.yaml names no return_face
    plan_hz: float = 5.
    sense_range: float = 3.5        # look for goals this close
    pad_hold: float = 0.12          # start the descent within this of the pad
    time_limit: float = 170.
    stall_s: float = 25.
    return_enabled: bool = True
    stop_after: str = None          # staged tests: land where it is and disarm once this goal passes
    pad_disarm: bool = False        # touch down on the pad armed, disarm only back on the launch mark
    pad_rest_s: float = 1.0         # on the pad before climbing again (under PX4's COM_DISARM_LAND)
    pad_lookout_reach: float = 0.2  # at a pad lookout within this
    pad_sweep_deg: float = 35.      # there, sweep the heading this far either side of the expected spot
    pad_sweep_s: float = 3.5        # ... once, over this long
    pad_look_aside_deg: float = 30. # on the way, turn the camera toward the mat's corner at most this far off the travel
                                    # direction (it must still see where it goes: a wall end it never saw is not mapped)
    pad_need: float = 0.45          # touch down this clear of anything mapped low beside the mat ...
    pad_reach: float = 0.1          # ... moving at most this far off the H for it: rocks stand >= 0.40 m off it,
                                    # all round, so moving off one approaches another (whose far side is unseen)
    handover_height: float = 1.0    # climb straight up (stiff hold) to here, then navigate: low down, the
                                    # stands' struts and feet crowd the start and the camera cannot see behind
    sill_clearance: float = 0.14    # through a window: skids this far over its sill
    window_rise: float = 0.31       # pass a window this far above its measured sill
    heading_until: str = 'gate_a'   # learn the course heading until this goal passes
    kind_speed: dict = field(default_factory=lambda: {'tube': 0.6, 'opening': 0.35, 'gap': 0.65, 'pad': 0.7})
    net_margin: float = 0.10        # keep-out inside the expected net
    line_net_margin: float = 0.03   # ... where the floor tape locates it
    repel_reach: float = 0.40       # push off anything mapped closer than this
    repel_gain: float = 3.0
    repel_limit: float = 0.6


@dataclass
class Setpoint:
    velocity: np.ndarray            # ENU m/s
    yaw: float                      # ENU radians
    position: np.ndarray = None     # ENU odometry: hold this point too (PX4 position loop)
    acceleration: np.ndarray = None # ENU m/s^2 feedforward


@dataclass
class Status:
    state: str = 'IDLE'
    reason: str = ''
    events: list = field(default_factory=list)


class GoalMission:
    def __init__(self, spec, config=None, plan_config=None, follow_config=None):
        self.spec = spec
        self.cfg = config or MissionConfig()
        self.planner = Planner(plan_config)
        self.follower = Follower(follow_config)
        self.window = WindowManeuver()
        self.status = Status()
        self.route = None
        self.map = None
        self.floor = None
        self.path = None
        self.speed = 0.
        self.free_ahead = 0.
        self.home = None
        self.hold_xy = None
        self.fields = None
        self.pad_samples = []
        self.pad_lookout = 0            # the lookout the drone looks for the mat from next (goals.yaml)
        self.pad_sweep_since = None     # sweeping at it since
        self.return_from = None         # where the return's straight line starts (the pad)
        self.climb_h = None             # the takeoff's sliding height target
        self.stopped_after = None       # the goal a staged test (stop_after) ended at
        self.dt_last = 0.02
        self.last_plan = -math.inf
        self.phase_since = 0.
        self.progress_at = 0.
        self.last_arm = -math.inf
        self.telemetry = None
        self.started = None

    # -- lifecycle -------------------------------------------------------------------
    @property
    def state(self):
        return self.status.state

    def _enter(self, state, now, reason=''):
        self.status.state, self.status.reason = state, reason
        self.status.events.append((round(now, 3), state, reason))
        self.phase_since = now

    def start(self, telemetry):
        """The single start command: build the map and route around the
        current pose. Refuses without valid, landed, disarmed telemetry."""
        if self.state != 'IDLE':
            return False, f'already {self.state}'
        if not telemetry.valid or not telemetry.landed or telemetry.armed:
            return False, 'needs valid, landed, disarmed telemetry'
        p = telemetry.position
        floor_z = p[2] - self.cfg.landed_base_height
        self.home = p[:2].copy()
        right, ahead = self.spec.get('return_offset', (0., 0.))
        c, s = math.cos(yaw_of(telemetry.rotation)), math.sin(yaw_of(telemetry.rotation))
        self.return_xy = self.home + right * np.array([s, -c]) + ahead * np.array([c, s])
        self.return_spot = None             # built with the route (below)
        # Climbing, held a little off where it sits (goals.yaml takeoff_offset,
        # [right, ahead]): the barrier is just behind the mark, the net to its left.
        t_right, t_ahead = self.spec.get('takeoff_offset', (0., 0.))
        self.takeoff_xy = p[:2] + t_right * np.array([s, -c]) + t_ahead * np.array([c, s])
        yaw = yaw_of(telemetry.rotation)
        self.route = Route(self.spec, p[:2], yaw)
        self.return_spot = ReturnSpot(self.route, self.return_xy, self.spec.get('return_walls', []))
        face = self.spec.get('return_face')
        self.return_face = self.route.cad_point(face) if face is not None else None
        self.map = LiveMap(self.route.net_centre(), floor_z)
        self.line_map = LineMap(self.map)
        self.floor = FloorTracker(floor_z)
        self._update_fences()
        self.follower.reset(yaw)
        self.started = telemetry.t
        self.progress_at = telemetry.t
        self._enter('PREPARE', telemetry.t)
        return True, 'started'

    def _update_fences(self):
        """The net, where the CAD puts it, with sides moved by the walls they
        are anchored to (``net_anchors`` in the goal file): the net is nearly
        invisible, the walls are not."""
        r = self.route
        hx, hy = r.net_half
        west, east, south, north = -hx, hx, -hy, hy
        shifts = getattr(self, 'net_shift', {})
        west, east = west + shifts.get('west', 0.), east + shifts.get('east', 0.)
        south, north = south + shifts.get('south', 0.), north + shifts.get('north', 0.)
        # Keep-out inside each side: smaller where the floor tape locates it.
        # Plus any course-specific extra (goals.yaml net_extra): along the first
        # corridor the drone keeps nearer M1 (mapped) than the net (nearly invisible).
        extra = self.spec.get('net_extra', {}) or {}
        m = {side: (self.cfg.line_net_margin if side == getattr(self, 'net_from_line', None) else self.cfg.net_margin)
             + float(extra.get(side, 0.)) for side in ('west', 'east', 'south', 'north')}
        fw, fe, fs, fn = west + m['west'], east - m['east'], south + m['south'], north - m['north']
        self.fence = Fence(tuple(r.cad_point(((fw + fe) / 2, (fs + fn) / 2))), ((fe - fw) / 2, (fn - fs) / 2), r.yaw)
        centre = tuple(r.cad_point(((west + east) / 2, (south + north) / 2)))
        half = ((east - west) / 2, (north - south) / 2)
        self.map.point_fence = Fence(centre, (half[0] + 0.5, half[1] + 0.5), r.yaw)
        self.map.carve_fence = Fence(centre, (half[0] - 0.1, half[1] - 0.1), r.yaw)

    def _anchor_net(self):
        floor = self.floor.floor_z
        shifts = getattr(self, 'net_shift', {})
        for anchor in self.spec.get('net_anchors', []):
            if anchor['side'] == getattr(self, 'net_from_line', None):
                continue                      # The tape anchors that side better.
            found = wall_offset(self.route, self.map, *anchor['wall'], floor + 0.35, floor + 0.75)
            if found is None:
                continue
            a, b = (np.asarray(p, float) for p in anchor['wall'])
            normal = np.array([-(b - a)[1], (b - a)[0]]) / np.linalg.norm(b - a)
            axis = 0 if anchor['side'] in ('west', 'east') else 1
            shift = found * normal[axis]                # the wall's move along the side's axis
            outward = 1. if anchor['side'] in ('north', 'east') else -1.
            shift = outward * min(outward * shift, 0.)      # A wall may only bring the net in: it is invisible.
            old = shifts.get(anchor['side'], 0.)
            shifts[anchor['side']] = old + 0.3 * (shift - old)
        self.net_shift = shifts
        self._update_fences()

    # -- perception ------------------------------------------------------------------------
    def add_depth(self, origin, points):
        if self.map is not None and self.state not in ('IDLE', 'DONE'):
            self.map.insert(origin, points)
            if self.state in ('RETURN', 'RETURN_DESCEND'):
                self._observe_return(origin, points)
            goal = self.route.current if self.route else None
            if goal is not None and goal.kind == 'opening' and self.window.active_for(goal):
                h = self.floor.floor_z + self._goal_height(goal)
                self.window.observe(points, goal, h - 0.12, h + 0.12)
                found = self.window.measured_centre()
                if found is not None:
                    goal.detected = found
                    self.route.update_estimates()
                sill = self.window.measured_sill()
                if sill is not None:
                    # The middle of even the smallest window (0.62 m tall) over its sill.
                    goal.pass_height = float(sill - self.floor.floor_z + self.cfg.window_rise)

    def _observe_return(self, origin, points):
        """Move the touchdown spot toward where the walls by the mark, seen
        now, put it (``return_spot``), a little at a time."""
        spot = self.return_spot
        if self.telemetry is None or self.floor.height(self.telemetry.position[2]) < 0.6:
            return                            # Low down it sees tarp skirts, not faces: keep the spot.
        was = spot.measured()
        spot.observe(np.asarray(origin, float), np.asarray(points, float), self.floor.floor_z)
        self.return_xy = self.return_xy + 0.2 * (spot.target() - self.return_xy)
        if not was and spot.measured():
            shift = spot.target() - spot.base
            self.status.events.append((round(self.telemetry.t, 3), 'return spot', f'{shift[0]:+.2f} {shift[1]:+.2f}'))

    def add_range(self, telemetry, distance):
        if self.floor is None or telemetry.landed:
            return
        tilt_cos = float(telemetry.rotation[2, 2])
        found = self.floor.update(telemetry.position[2], distance, tilt_cos, self._floor_clear(telemetry))
        if found and self.state == 'NAVIGATE':
            self.map.mark(telemetry.position[:2], found[1])

    def _floor_clear(self, telemetry, radius=0.3):
        """No mapped obstacle under the rangefinder's footprint: only then is
        its reading the floor (tarp skirts, cradles and boulders are not)."""
        m = self.map
        ij = m.cell(telemetry.position[:2])
        r = int(radius / m.res)
        if not m.inside(ij):
            return False
        top = max(1, int(m.band_of(telemetry.position[2] - 0.15)))
        block = m.logodds[:top, max(ij[0] - r, 0):ij[0] + r + 1, max(ij[1] - r, 0):ij[1] + r + 1]
        return not bool((block > 0.).any())

    def add_line(self, points):
        if self.state == 'NAVIGATE' and len(points):
            self.line_map.add(points)

    def wants_line(self):
        return self.route is not None and any(g.line_turn for g in self.route.upcoming(3))

    def _line_turn(self, goal, past=0.):
        """Put a ``line_turn`` goal where the floor line turns (CP1): judged by
        eye, reaching it is enough, and that line is only 0.46 m from the net."""
        side = 1 if goal.line_turn == 'left' else -1
        found = turn_along(self.line_map, goal.prior, self.route.direction(goal), side)
        if found is None:
            return False
        d = self.route.direction(goal)
        across = 0. if goal.detected is None else float((goal.detected - goal.prior) @ np.array([-d[1], d[0]]))
        goal.detected = goal.prior + d * (found + past) + np.array([-d[1], d[0]]) * across
        goal.line_seen = True
        if goal.turn_to_net is not None:
            # The net ahead: its CAD distance past the turn, from where the tape turns.
            a = math.radians(goal.heading_deg)
            side = 'north' if math.sin(a) > 0.7 else 'south' if math.sin(a) < -0.7 else 'east' if math.cos(a) > 0 else 'west'
            # Both in the odometry frame: the net is turn_to_net past the turn,
            # along the travel direction; compare with where the fence puts it.
            turn = goal.prior + d * found
            net_here = turn + d * goal.turn_to_net
            axis = 1 if side in ('north', 'south') else 0
            sign = 1 if side in ('north', 'east') else -1
            nominal = np.zeros(2)
            nominal[axis] = sign * self.route.net_half[axis]
            nominal[1 - axis] = float(np.asarray(goal.cad)[1 - axis])
            shift = float((net_here - self.route.cad_point(nominal)) @ d) * sign
            self.net_shift = {**getattr(self, 'net_shift', {}), side: sign * shift}
            self.net_from_line = side
            self._update_fences()
        return True

    def add_pad(self, xy):
        if self.route is None:
            return
        xy = np.asarray(xy, float)
        if not self.fence.contains(xy[None], margin=-0.2)[0]:
            return          # A mat beyond the net: the next course's, seen through it.
        self.pad_samples = (self.pad_samples + [xy])[-15:]

    # -- planning ---------------------------------------------------------------------------
    def _band(self, height, low_height=None, tight=False):
        """Heights the body sweeps between ``low_height`` and ``height`` (changing
        level). ``tight``: at a window, over its flat sill, without the margin
        below kept for boulders."""
        floor = self.floor.floor_z
        low_height = height if low_height is None else min(low_height, height)
        below, above = (0.03, 0.02) if tight else (self.cfg.vertical_margin, 0.05)
        goal = self.route.current if self.route else None
        if goal is not None and goal.kind == 'tube':
            # Under a mouth's hanging tarp: 5 cm would reach the next 10 cm map
            # band (1.0-1.1 m at 0.8 m), where the tarp narrows the mouth to ~0.45 m.
            above = 0.02
        return (floor + low_height - self.cfg.landed_base_height - below,
                floor + height + self._body_above() + above)

    def _body_above(self):
        """The body's top (the battery) over its origin."""
        return self.cfg.body_height - self.cfg.landed_base_height

    def _at_window(self):
        goal = self.route.current if self.route else None
        if goal is None:
            return False
        anchor = next((g for g in self.route.goals if g.id == goal.anchor), None) if goal.anchor else None
        return goal.kind == 'opening' or (anchor is not None and anchor.kind == 'opening')

    def _fields(self, height, low_height=None):
        return self._fields_between(*self._band(height, low_height, tight=self._at_window()))

    def _fields_between(self, low, high):
        """(blocked, clearance, known) in the map between two odometry heights,
        everything outside the net fence blocked."""
        blocked = self.map.blocked(low, high)
        n = self.map.n
        centres = self.map.origin + (np.indices((n, n)).transpose(1, 2, 0) + 0.5) * self.map.res
        blocked = blocked | ~self.fence.contains(centres)
        a, b = self.map.bands_between(low, high)
        known = (self.map.logodds[a:b] != 0).any(axis=0)
        return blocked, clearance(blocked, self.map.res), known

    def _detection_fields(self, goal):
        """(clearance, known) to find a window in: a band in the middle of
        the opening. The body's passing band at the default height can reach
        the lintel of a low, short window (in 10 cm map bands) and fill the
        hole; the pass height is measured from the sill once it is found."""
        floor, h = self.floor.floor_z, goal.pass_height or goal.height or self.cfg.cruise_height
        return self._fields_between(floor + h - 0.18, floor + h + 0.05)[1:]

    def plan(self, telemetry):
        cfg, route = self.cfg, self.route
        self.last_plan = telemetry.t
        p = telemetry.position[:2]
        if not self._heading_frozen():
            # The course's heading from long walls fitted as lines (a grid's
            # gradients lean toward its own axes and cannot measure a few degrees).
            floor = self.floor.floor_z
            fits = [wall_rotation(route, self.map, *w, floor + 0.35, floor + 1.4) for w in self.spec.get('heading_walls', [])]
            fits = [f for f in fits if f is not None]
            if fits:
                angle = sum(a * w for a, w in fits) / sum(w for _, w in fits)
                route.update_heading(route.heading_correction + angle)
                self._update_fences()
        self._anchor_net()
        goal = route.current
        height = self._goal_height(goal)
        now = self.floor.height(telemetry.position[2])
        blocked, clear, known = self._fields(max(height, now), min(height, now))   # all it sweeps on the way
        self.fields = (blocked, clear, known)
        for g in route.upcoming(3):
            route.update_estimates()
            if g.kind == 'pad':
                if len(self.pad_samples) >= 3:
                    g.detected = np.median(np.array(self.pad_samples), axis=0)
                    g.landing = self._landing_spot(g.detected)
                continue
            if g.line_turn and self._line_turn(g):
                continue                      # The tape says exactly where; the map need not guess.
            if g.kind == 'opening' and self.window.active_for(g) and self.window.phase != 'settle':
                continue                      # Measured from the settle point; close up the jambs leave the view.
            if g.kind == 'opening' and g.detected is None and np.linalg.norm(g.estimate - p) < cfg.sense_range:
                self._snap_to_wall(g)
            if g.anchor is None and np.linalg.norm(g.estimate - p) < cfg.sense_range:
                fields = (clear, known)
                if g.kind == 'opening' and g.detected is None:
                    fields = self._detection_fields(g)
                if refine(g, route, *fields, self.map) and g.kind == 'opening':
                    # Square the window to its panel beside the opening (above
                    # the stands' struts and feet, below what the camera cannot see).
                    floor = self.floor.floor_z
                    opening_axis(g, route, self.map, floor + 0.50, floor + 0.95)
                    self._opening_height(g)
                    self._opening_centre(g)
        route.update_estimates()
        targets = [g.estimate for g in route.upcoming(2)]
        if goal is not None and goal.kind in ('pad', 'reach'):
            targets = [goal.estimate]            # Arrive there; the lookahead would turn early.
        if goal is not None and goal.kind == 'pad' and goal.detected is None and self.pad_lookout < len(goal.lookouts):
            targets = [route.lookout(goal, self.pad_lookout)]      # Not seen yet: to where it can be.
        tight = [(g.estimate, g.axis, 0.45) for g in route.upcoming(3)
                 if g.kind == 'opening' and g.detected is not None and g.axis is not None]
        legs = route.upcoming(2) if not (goal is not None and goal.kind in ('pad', 'reach')) else [goal]
        confine = [g.confine for g in legs][:len(targets)]
        exact = [g.kind == 'reach' for g in legs][:len(targets)]
        self.path = self.planner.plan(self.map, clear, known, p, targets, tight, confine, exact) if targets else None
        if self.path is not None:
            turn = heading_change([np.asarray(q) for q in self.path], p)
            self.speed, self.free_ahead = self.planner.speed_limit(self.path, self.map, clear, known, p, turn)
            if self.planner.cfg.speed_caps:
                self.speed = min(self.speed, self._goal_speed(p))
        else:
            self.speed, self.free_ahead = 0., 0.

    def _goal_speed(self, p, deceleration=0.8):
        """Each kind of goal has a speed to pass it at (slow through the
        tunnel and the window); brake toward it in time."""
        limit = math.inf
        route = self.route
        recent = route.goals[max(route.index - 1, 0):route.index + 2]
        for g in recent:
            cap = self.cfg.kind_speed.get(g.kind)
            if cap is None:
                continue
            d = float(np.linalg.norm(g.estimate - p))
            if g.passed is not None and d > 0.6:
                continue                            # Well clear of it.
            flat = 0.8 if g.kind in ('opening', 'tube') else 0.5 if g.kind == 'reach' else 0.3
            # A dead end (reach) brakes gently and early: its goal moves nearer
            # when the floor line is seen, close in, and the net is just past it.
            rate = deceleration / 2 if g.kind == 'reach' else deceleration
            limit = min(limit, math.sqrt(cap ** 2 + 2 * rate * max(0., d - flat)))
            if g.kind == 'opening' and g.passed is None and d < 0.9 and self.telemetry is not None:
                # Level first, then in: never reach the wall still descending.
                error = abs(self.floor.height(self.telemetry.position[2]) - self._goal_height(g))
                if error > 0.05:
                    limit = min(limit, 0.05)
        return limit

    def _heading_frozen(self):
        passed = {g.id for g in self.route.goals if g.passed is not None}
        return self.cfg.heading_until in passed

    def _goal_height(self, goal):
        if goal is None:
            return self.cfg.cruise_height
        anchor = next((g for g in self.route.goals if g.id == goal.anchor), None) if goal.anchor else None
        window = goal if goal.kind == 'opening' else anchor
        if window is not None and window.kind == 'opening' and window.detected is None:
            # Not found yet: look from cruise height, where the tilted camera sees
            # down into the opening; low down, close in, it sees only the panel.
            return self.cfg.cruise_height
        if anchor is not None and (anchor.pass_height or anchor.height):
            return anchor.pass_height or anchor.height   # Approach and leave a window at its height.
        return goal.pass_height or goal.height or self.cfg.cruise_height

    def _snap_to_wall(self, goal, reach=1.0):
        """Put a window's prior on its wall: fit the panel near it (beside the
        opening) as a line and slide the prior onto that line, so the search
        along the wall for the opening looks where the wall really is."""
        floor = self.floor.floor_z
        cells = np.argwhere(self.map.blocked(floor + 0.50, floor + 0.95))
        fit = wall_line(self.map.centre_of(cells), goal.prior, self.route.direction(goal), reach, 10)
        if fit is None:
            return
        centre, along = fit
        normal = np.array([-along[1], along[0]])
        if abs(normal @ self.route.direction(goal)) < math.cos(math.radians(35)):
            return
        goal.prior = goal.prior - normal * float((goal.prior - centre) @ normal)

    def _opening_centre(self, goal, reach=0.6):
        """Centre a window between its jambs: walk along the wall line at the
        pass height from the best-clearance point to the first occupied cell
        each side; the midpoint is good to a few cm (best clearance alone can
        sit 10+ cm off in an opening barely wider than the drone)."""
        if goal.axis is None or goal.detected is None:
            return
        height = self._goal_height(goal)
        low, high = self._band(height)
        blocked = self.map.blocked(low, high)
        along = np.array([-goal.axis[1], goal.axis[0]])
        edges = []
        for sign in (1., -1.):
            for s in np.arange(0.025, reach, 0.025):
                hit = False
                for t in (-0.04, 0., 0.04):               # through the panel's thickness
                    ij = self.map.cell(goal.detected + along * sign * s + goal.axis * t)
                    if self.map.inside(ij) and blocked[ij[0], ij[1]]:
                        hit = True
                if hit:
                    edges.append(sign * s)
                    break
        if len(edges) == 2 and 0.45 < edges[0] - edges[1] < 1.2:
            shift = (edges[0] + edges[1]) / 2
            goal.detected = goal.detected + along * shift
            goal.opening_width = edges[0] - edges[1]

    def _opening_height(self, goal):
        """Pass a window just above its sill (seen below the camera's level):
        the body's underside ``sill_clearance`` over it. Its top is above the
        tilted camera's view, so it is not used."""
        floor = self.map.floor_z
        d = self.route.direction(goal) if goal.axis is None else goal.axis
        along = np.array([-d[1], d[0]])
        # The free gap is looked for around the expected pass height: the
        # middle of a fixed 0.1-1.4 m span (0.75 m) is solid panel under a
        # high window, and its "gap" a bogus sill.
        expected = goal.height or self.cfg.cruise_height
        sills, lintels = [], []
        for s in np.arange(-0.15, 0.151, 0.05):           # across the opening's width
            for t in (-0.05, 0., 0.05):                     # and through the panel's thickness
                ij = self.map.cell(goal.detected + along * s + d * t)
                if self.map.inside(ij):
                    bottom, top = self.map.free_interval(ij, floor + expected - 0.6, floor + expected + 0.6)
                    sills.append(bottom - floor)
                    lintels.append(top - floor)
        if not sills:
            return
        sill, lintel = max(sills), min(lintels)   # band boundaries: the tightest column
        if not 0.2 < sill < expected - 0.1:
            return
        if sill <= expected - 0.6:
            return                  # Nothing under it in the span looked at: open air, not over a sill.
        # The obstacle check spans feet + vertical_margin below the origin and
        # rotor tops + 0.05 above: centre that span in the opening.
        low = sill + self.cfg.landed_base_height + self.cfg.vertical_margin + 0.02
        high = lintel - self._body_above() - 0.05 - 0.02
        if lintel >= expected + 0.55:          # not seen (above the camera): stay just over the sill
            high = low
        height = (low + high) / 2 if high >= low else low
        goal.pass_height = float(min(height, low + 0.15) + (self.map.floor_z - self.floor.floor_z))

    # -- control -----------------------------------------------------------------------------
    def step(self, telemetry, dt):
        """One control tick: (Setpoint or None, [PX4 commands])."""
        self.telemetry = telemetry
        self.dt_last = dt
        state, now = self.state, telemetry.t
        if state in ('IDLE', 'DONE', 'ABORTED'):
            return None, []
        if not telemetry.valid:
            return self._abort(now, 'position estimate invalid')
        if self.started is not None and now - self.started > self.cfg.time_limit:
            return self._abort(now, 'time limit')
        return getattr(self, '_' + state.lower())(telemetry, dt)

    def _abort(self, now, reason):
        self._enter('ABORTED', now, reason)
        return None, [(LAND, 0., 0.)]

    def _hold(self, telemetry, vz=0.):
        yaw = self.follower.yaw if self.follower.yaw is not None else yaw_of(telemetry.rotation)
        return Setpoint(np.array([0., 0., vz]), yaw)

    def _prepare(self, t, dt):
        commands = []
        if t.t - self.phase_since > 0.5 and t.t - self.last_arm > 1.:
            self.last_arm = t.t
            commands = [(SET_MODE, 1., 6.), (ARM, 1., 0.)]
        if t.armed and t.offboard:
            self._enter('TAKEOFF', t.t)
        return self._hold(t), commands

    def _climb_to(self, t, height, xy):
        """Straight up over ``xy``, held there by PX4's position loop: the walls
        behind the drone are out of the camera's view, so nothing else keeps
        it off them (a velocity hold let it drift 14 cm onto the barrier)."""
        h = self.floor.height(t.position[2])
        vz = self.follower.climb(h, height)
        if self.climb_h is None:
            self.climb_h = h
        self.climb_h = min(self.climb_h + max(vz, 0.) * self.dt_last, height)
        position = np.array([*xy, self.floor.floor_z + max(self.climb_h, h)])
        return Setpoint(np.array([0., 0., vz]), self.follower.yaw, position=position,
                        acceleration=np.array([*self._stiff(xy, t), 0.])), abs(height - h)

    @staticmethod
    def _stiff(xy, t, gain=6.0, damping=3.5, limit=2.0):
        """Horizontal acceleration feedforward holding a point (as the window's
        lateral loop: wall and rock suction pull at ~0.5 m/s^2 near them)."""
        accel = gain * (np.asarray(xy, float) - t.position[:2]) - damping * t.velocity[:2]
        norm = float(np.linalg.norm(accel))
        return accel * (limit / norm) if norm > limit else accel

    def _repel(self, t):
        """Push away from anything mapped closer than ``repel_reach`` (a reflex
        under the planner: wall suction and gusts act faster than a replan)."""
        gain, reach, limit = self.cfg.repel_gain, self.cfg.repel_reach, self.cfg.repel_limit
        if t.t - getattr(self, 'repel_at', -1.) > 0.2 or getattr(self, 'seen_clear', None) is None:
            h = max(self.floor.height(t.position[2]), 0.25)
            low, high = self._band(h)
            # Only what was seen: pushing off the estimated (invisible) net can
            # push the drone into a wall beside it that the camera cannot see.
            self.seen_clear = clearance(self.map.blocked(low, high), self.map.res)
            self.repel_at = t.t
        clear = self.seen_clear
        ij = self.map.cell(t.position[:2])
        if not self.map.inside(ij) or not (1 <= ij[0] < self.map.n - 1 and 1 <= ij[1] < self.map.n - 1):
            return np.zeros(2)
        c = clear[ij[0], ij[1]]
        if c >= reach:
            return np.zeros(2)
        grad = np.array([clear[ij[0] + 1, ij[1]] - clear[ij[0] - 1, ij[1]],
                         clear[ij[0], ij[1] + 1] - clear[ij[0], ij[1] - 1]])
        norm = float(np.linalg.norm(grad))
        if norm < 1e-6:
            return np.zeros(2)
        return grad / norm * min(limit, gain * (reach - c))

    def _takeoff(self, t, dt):
        setpoint, error = self._climb_to(t, self.cfg.cruise_height, self.takeoff_xy)
        if self.floor.height(t.position[2]) > 0.15:
            push = self._repel(t)
            self.takeoff_xy = self.takeoff_xy + push * dt      # Move the hold point with it.
        if self.floor.height(t.position[2]) > self.cfg.handover_height:
            self._enter('NAVIGATE', t.t)        # Clear of the skids' ground effect: let the planner centre it.
        return setpoint, []

    def _navigate(self, t, dt):
        route = self.route
        if route.advance_if_passed(t.position[:2], t.t):
            done = route.goals[route.index - 1]
            self.status.events.append((round(t.t, 3), 'passed', done.id))
            self.progress_at = t.t
            if done.id == self.cfg.stop_after:
                # A staged test ends here: straight down where it is, then disarm.
                self.stopped_after = done.id
                self.pad_xy, self.pad_descend_h = t.position[:2].copy(), None
                self._enter('PAD_DESCEND', t.t, f'stopping after {done.id}')
                return self._pad_descend(t, dt)
        goal = route.current
        if goal is None:
            return self._abort(t.t, 'route ended without the pad')
        if t.t - self.progress_at > self.cfg.stall_s:
            return self._abort(t.t, f'no progress toward {goal.id}')
        if t.t - self.last_plan >= 1. / self.cfg.plan_hz:
            self.plan(t)
        searching = goal.kind == 'pad' and goal.detected is None
        if searching:
            setpoint = self._pad_search(t, dt, goal)
            if setpoint is not None:
                return setpoint
        elif goal.kind == 'pad':
            offset = getattr(goal, 'landing', goal.estimate) - t.position[:2]
            if np.linalg.norm(offset) < 0.6:
                return self._pad_approach(t, dt, goal, offset)
        previous = route.goals[route.index - 1] if route.index > 0 else None
        if goal.kind == 'opening' and goal.axis is not None and goal.detected is not None \
                and previous is not None and previous.passed is not None:
            return self._through_window(t, dt, goal)
        dead_end = next((g for g in route.goals[:route.index] if g.kind == 'reach' and g.passed is not None
                         and t.t - g.passed < 2.), None)
        if dead_end is not None and np.linalg.norm(t.velocity[:2]) > 0.05:
            return self._settle(t, dead_end.estimate)
        yaw_now = yaw_of(t.rotation)
        yaw_before = self.follower.yaw if self.follower.yaw is not None else yaw_now
        velocity, yaw = self.follower.step(dt, self.path, t.position[:2], self.speed, yaw_now)
        if searching:                         # On the way to a lookout: the camera toward where the mat may be.
            face = math.atan2(*(goal.estimate - t.position[:2])[::-1])
            if np.linalg.norm(velocity) > 0.1:
                travel, aside = math.atan2(velocity[1], velocity[0]), math.radians(self.cfg.pad_look_aside_deg)
                face = travel + float(np.clip(wrap(face - travel), -aside, aside))
            self.follower.yaw = yaw_before
            yaw = self._turn_to(face, dt)
        push = self._repel(t)
        velocity = self._hold_if_stopped(t, velocity) + push
        if self.hold_xy is not None:
            self.hold_xy = self.hold_xy + push * dt      # Never hold a point the reflex pushes off.
        height = self._goal_height(goal)
        vz = self.follower.climb(self.floor.height(t.position[2]), height)
        if self.hold_xy is None and self.path is not None and getattr(self.follower, 'nearest', None) is not None:
            # Moving along a path: a full setpoint holds its line far more tightly.
            accel = self.follower.track_accel(t.position[:2], t.velocity[:2])
            error = self.floor.height(t.position[2]) - height
            az = float(np.clip(-4.0 * error - 3.0 * t.velocity[2], -2., 2.))
            position = np.array([*self.follower.nearest, self.floor.floor_z + height])
            return Setpoint(np.array([*velocity, vz]), yaw, position=position, acceleration=np.array([*accel, az])), []
        if self.hold_xy is not None:
            # Stopped (the planner allows almost no speed): hold the point with a
            # full setpoint; a velocity hold let wall suction pull it 9 cm onto
            # the barrier just after takeoff.
            error = self.floor.height(t.position[2]) - height
            az = float(np.clip(-4.0 * error - 3.0 * t.velocity[2], -2., 2.))
            position = np.array([*self.hold_xy, self.floor.floor_z + height])
            return Setpoint(np.array([*push, vz]), yaw, position=position,
                            acceleration=np.array([*self._stiff(self.hold_xy, t), az])), []
        return Setpoint(np.array([*velocity, vz]), yaw), []

    def _through_window(self, t, dt, goal):
        """Lined up in front of a measured window: the window manoeuvre flies it."""
        if not self.window.active_for(goal):
            self.window.start(t.t, goal)
            self.status.events.append((round(t.t, 3), 'window', 'settle'))
        height = self._goal_height(goal)
        error = self.floor.height(t.position[2]) - height
        target, feedforward, accel, yaw, phase = self.window.step(t.t, dt, t.position[:2], t.velocity[:2], error, goal)
        turn = float(np.clip(wrap(yaw - self.follower.yaw), -self.follower.cfg.yaw_rate * dt,
                             self.follower.cfg.yaw_rate * dt))
        self.follower.yaw = wrap(self.follower.yaw + turn)
        self.follower.velocity = feedforward.copy()
        az = float(np.clip(-6.0 * error - 3.5 * t.velocity[2], -2., 2.))
        position = np.array([*target, self.floor.floor_z + height])
        return Setpoint(np.array([*feedforward, 0.]), self.follower.yaw, position=position,
                        acceleration=np.array([*accel, az])), []

    def _settle(self, t, xy):
        """Stop at a dead end (a ``reach`` goal, the net just past it) before
        turning away: a stiff hold there, not the turn's path, which starts
        where the drone is and lets it coast on toward the net."""
        height = self.cfg.cruise_height
        az = float(np.clip(-4.0 * (self.floor.height(t.position[2]) - height) - 3.0 * t.velocity[2], -2., 2.))
        return Setpoint(np.zeros(3), self.follower.yaw, position=np.array([*xy, self.floor.floor_z + height]),
                        acceleration=np.array([*self._stiff(xy, t), az])), []

    def _hold_if_stopped(self, t, velocity, gain=2.5, damping=0.6, limit=0.4):
        """A velocity setpoint alone lets turbulence and wall suction push the
        drone around; while the planner allows almost no speed, hold the
        position where it stopped, stiffly and damped (wall pull near a
        panel is ~0.5 m/s^2, faster than a soft hold reacts)."""
        if self.speed > 0.08 and self.path is not None:
            self.hold_xy = None
            return velocity
        if self.hold_xy is None:
            self.hold_xy = t.position[:2].copy()
        pull = gain * (self.hold_xy - t.position[:2]) - damping * t.velocity[:2]
        norm = float(np.linalg.norm(pull))
        return pull * (limit / norm) if norm > limit else pull

    def _landing_spot(self, centre):
        """Where on the mat to touch down: the H at its centre. Only if
        something mapped low down beside it (a wall's tarp, its stand's
        struts, a rock) is within ``pad_need`` (rotor tips 0.29 m, the
        touchdown error, and some of a rock's far side, which the camera
        may not see), the nearest spot that clears it, at most
        ``pad_reach`` off the centre (the rotors stay on the mat)."""
        centre, need, reach = np.asarray(centre, float), self.cfg.pad_need, self.cfg.pad_reach
        floor = self.floor.floor_z
        clear = clearance(self.map.blocked(floor + 0.06, floor + 0.6), self.map.res)
        best, score = centre, -math.inf
        for dx in np.arange(-reach, reach + 1e-9, 0.025):
            for dy in np.arange(-reach, reach + 1e-9, 0.025):
                q = centre + np.array([dx, dy])
                ij = self.map.cell(q)
                if not self.map.inside(ij) or not self.fence.contains(q[None], margin=-0.4)[0]:
                    continue                  # Rotor tips 0.1 m clear of the (invisible) net.
                s = min(float(clear[ij[0], ij[1]]), need) - 0.01 * math.hypot(dx, dy)
                if s > score:
                    best, score = q, s
        return best

    def _pad_approach(self, t, dt, goal, offset):
        distance = float(np.linalg.norm(offset))
        speed = min(0.6, 1.2 * distance)
        wanted = offset / max(distance, 1e-6) * speed
        self.follower.velocity = self.follower.velocity + np.clip(wanted - self.follower.velocity, -0.05, 0.05)
        vz = self.follower.climb(self.floor.height(t.position[2]), self.cfg.cruise_height)
        if distance < self.cfg.pad_hold and np.linalg.norm(t.velocity[:2]) < 0.2:
            goal.passed = t.t
            self.pad_xy = np.array(getattr(goal, 'landing', goal.estimate), float)
            self.pad_descend_h = None
            self._enter('PAD_DESCEND', t.t)
        return Setpoint(np.array([*self.follower.velocity, vz]), self.follower.yaw), []

    def _pad_search(self, t, dt, goal):
        """The mat not seen yet: fly to each of the goal's lookouts in turn
        (the planned path, the camera kept on the expected spot) and sweep the
        heading once there. None while on the way; aborts when no lookout
        shows it (never hovers looking forever)."""
        if self.pad_lookout >= len(goal.lookouts):
            return self._abort(t.t, 'pad not seen from any lookout')
        look = self.route.lookout(goal, self.pad_lookout)
        if self.pad_sweep_since is None:
            off = float(np.linalg.norm(look - t.position[:2]))
            if off > self.cfg.pad_lookout_reach and not (off < 0.5 and self.speed < 0.08):
                return None                   # On the way (or stopped just short of it: look from there).
            self.pad_sweep_since = t.t
            self.status.events.append((round(t.t, 3), 'pad lookout', str(self.pad_lookout)))
        phase = (t.t - self.pad_sweep_since) / self.cfg.pad_sweep_s
        if phase >= 1.:
            self.pad_lookout, self.pad_sweep_since, self.progress_at = self.pad_lookout + 1, None, t.t
        face = math.atan2(*(goal.estimate - look)[::-1])
        yaw = self._turn_to(face + math.radians(self.cfg.pad_sweep_deg) * math.sin(2 * math.pi * min(phase, 1.)), dt)
        error = self.floor.height(t.position[2]) - self.cfg.cruise_height
        az = float(np.clip(-4.0 * error - 3.0 * t.velocity[2], -2., 2.))
        position = np.array([*look, self.floor.floor_z + self.cfg.cruise_height])
        return Setpoint(np.array([*self._repel(t), 0.]), yaw, position=position,
                        acceleration=np.array([*self._stiff(look, t), az])), []

    def _turn_to(self, yaw, dt):
        """The follower's yaw, turned toward ``yaw`` at its rate limit."""
        rate = self.follower.cfg.yaw_rate * dt
        self.follower.yaw = wrap(self.follower.yaw + float(np.clip(wrap(yaw - self.follower.yaw), -rate, rate)))
        return self.follower.yaw

    def _pad_descend(self, t, dt):
        """Straight down onto the landing spot, its position held by PX4's
        position loop (a velocity setpoint alone let it drift 0.15 m toward a
        rock beside the mat), the height target sliding down."""
        h = self.floor.height(t.position[2])
        rate = 0.5 if h > 0.25 else 0.3
        if t.landed:
            self._enter('ON_PAD', t.t)
            disarm = self.cfg.pad_disarm or not self.cfg.return_enabled or self.stopped_after is not None
            return self._hold(t), [(ARM, 0., 0.)] if disarm else []
        if getattr(self, 'pad_descend_h', None) is None or self.phase_since == t.t:
            self.pad_descend_h = h
        self.pad_descend_h = max(self.pad_descend_h - rate * dt, -0.1)
        accel = self._stiff(self.pad_xy, t)
        position = np.array([*self.pad_xy, self.floor.floor_z + self.pad_descend_h])
        return Setpoint(np.array([0., 0., -rate]), self.follower.yaw, position=position,
                        acceleration=np.array([*accel, 0.])), []

    def _on_pad(self, t, dt):
        commands = []
        stopped = self.stopped_after is not None
        disarm = self.cfg.pad_disarm or not self.cfg.return_enabled or stopped
        if disarm and t.armed and t.t - self.phase_since < 0.5:
            commands = [(ARM, 0., 0.)]
        if not self.cfg.return_enabled or stopped:
            if not t.armed:
                self._enter('DONE', t.t, f'stopped after {self.stopped_after}' if stopped else 'landed on the pad')
            return self._hold(t), commands
        if t.armed and t.t - self.phase_since > (self.cfg.pad_rest_s if not disarm else 1.):
            self._enter('RETURN', t.t)
            self.return_from = None
        elif not t.armed and t.t - self.phase_since > 1. and t.landed and t.preflight_ok \
                and t.t - self.last_arm > 1.:
            self.last_arm = t.t
            commands = [(ARM, 1., 0.)]
        return self._hold(t), commands

    def _return(self, t, dt):
        """Back to the launch mark in one move: along the straight line, the
        height following a profile over everything mapped near it (unseen
        cells count as wall-tall), so it climbs while it moves off the pad,
        crosses the barrier high and comes down as soon as it is past; the
        last of the descent is straight down (RETURN_DESCEND). Full setpoints
        hold it to the line: the net is just past the mark."""
        cfg = self.cfg
        if self.return_from is None:
            self.return_from, self.return_profile_at = t.position[:2].copy(), -math.inf
        start, target = self.return_from, self.return_xy
        line = target - start
        length = max(float(np.linalg.norm(line)), 1e-6)
        tangent = line / length
        p, v = t.position[:2], t.velocity[:2]
        s = float(np.clip((p - start) @ tangent, 0., length))
        nearest = start + tangent * s
        left = float((target - p) @ tangent)
        if t.t - self.return_profile_at > 0.2:            # The camera keeps mapping on the way.
            self.return_profile, self.return_profile_at = self._return_profile(start, tangent, length), t.t
        grid, profile = self.return_profile
        h = self.floor.height(t.position[2])
        wanted = float(np.interp(np.clip(s + np.array([0., 0.15, 0.3, 0.45]), 0., length), grid, profile).max())
        speed = min(cfg.return_speed, math.sqrt(2. * cfg.return_brake * max(left - self._return_short(t, left), 0.)))
        below = wanted - h
        if below > 0.05:                  # Climbing: no faster along than the climb keeps up with.
            speed = min(speed, 0.8 * self.follower.cfg.max_climb / cfg.return_slope)
        if below > 0.25:
            speed = 0.                    # Well under the profile: climb in place first.
        normal = np.array([-tangent[1], tangent[0]])
        off, v_off = float((nearest - p) @ normal), float(v @ normal)
        along_error = float((v - tangent * speed) @ tangent)
        accel = normal * (4.0 * off - 3.0 * v_off) - tangent * 1.5 * along_error
        vz = self.follower.climb(h, wanted)
        az = float(np.clip(-4.0 * (h - wanted) - 3.0 * (t.velocity[2] - vz), -2., 2.))
        position = np.array([*(nearest if left > 0.05 else target), self.floor.floor_z + wanted])
        self.follower.yaw = self._return_yaw(p)
        if np.linalg.norm(target - p) < 0.06 and np.linalg.norm(v) < 0.08:
            self.descend_h = h
            self._enter('RETURN_DESCEND', t.t)
        return Setpoint(np.array([*(tangent * speed), vz]), self.follower.yaw, position=position,
                        acceleration=np.array([*np.clip(accel, -2., 2.), az])), []

    def _return_short(self, t, left, short=0.4, wait_s=3.):
        """How far short of the touchdown spot to stop: ``short`` until the
        walls by the mark are measured (the remembered spot is off by the
        drift, the net just past it), for at most ``wait_s`` stopped there."""
        if self.return_spot.measured() or not self.return_spot.walls:
            return 0.
        if left > short + 0.05:
            self.return_wait_since = None
        elif getattr(self, 'return_wait_since', None) is None:
            self.return_wait_since = t.t
        return short if self.return_wait_since is None or t.t - self.return_wait_since < wait_s else 0.

    def _return_yaw(self, p):
        """Facing the corner by the launch mark (goals.yaml ``return_face``):
        two walls in view say most about where the drone is as it lands.
        Close in, the heading from the mark itself, so it does not swing."""
        if self.return_face is None:
            return wrap(self.route.yaw + math.radians(self.cfg.return_yaw_deg))
        origin = p if np.linalg.norm(self.return_face - p) > 0.9 else self.return_xy
        d = self.return_face - origin
        return math.atan2(d[1], d[0])

    def _return_profile(self, start, tangent, length, step=0.05):
        """(s, height) along the return line: the tallest thing mapped within
        ``return_reach`` of each point, feet ``return_margin`` over it, then
        limited to ``return_slope`` up and down so the drone gets there in time."""
        cfg, m = self.cfg, self.map
        occupied = m.logodds > OCCUPIED_LOGODDS
        top_band = m.bands - 1 - np.argmax(occupied[::-1], axis=0)
        top = np.where(occupied.any(axis=0), m.floor_z + (top_band + 1) * m.band - self.floor.floor_z, 0.)
        # The tilted camera never sees above its own height, so a wall's
        # mapped top is short of its real one: anything over a rock's height
        # (0.53 m) is a wall or the tunnel, as tall as one.
        top = np.where(top > 0.7, np.maximum(top, cfg.return_unseen_top), top)
        top = np.where(m.seen, top, cfg.return_unseen_top)
        r = int(math.ceil(cfg.return_reach / m.res))
        di, dj = np.mgrid[-r:r + 1, -r:r + 1]
        disk = np.hypot(di, dj) * m.res <= cfg.return_reach
        di, dj = di[disk], dj[disk]
        s = np.arange(0., length + step, step)
        need = np.zeros(len(s))
        for k, q in enumerate(start + tangent * s[:, None]):
            i, j = m.cell(q)
            ii, jj = np.clip(i + di, 0, m.n - 1), np.clip(j + dj, 0, m.n - 1)
            need[k] = top[ii, jj].max() + cfg.landed_base_height + cfg.return_margin
        profile = (need[None, :] - cfg.return_slope * np.abs(s[:, None] - s[None, :])).max(axis=1)
        return s, np.clip(profile, 0., cfg.return_height)

    def _return_descend(self, t, dt):
        """Straight down onto the return mark: position held stiffly (PX4's
        position loop), the height target sliding down."""
        h = self.floor.height(t.position[2])
        rate = 0.6 if h > 0.6 else 0.3
        if t.landed:
            self._enter('DONE', t.t, 'returned to the launch')
            return None, [(ARM, 0., 0.)]
        self.descend_h = max(self.descend_h - rate * dt, -0.1)
        accel = self._stiff(self.return_xy, t)
        position = np.array([*self.return_xy, self.floor.floor_z + self.descend_h])
        return Setpoint(np.array([0., 0., -rate]), self.follower.yaw, position=position,
                        acceleration=np.array([*accel, 0.])), []
