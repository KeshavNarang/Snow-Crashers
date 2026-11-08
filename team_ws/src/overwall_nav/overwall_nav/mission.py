"""Take off, creep forward until a wall is 0.5 m ahead, climb over it, land behind it.

No ROS here (tested on its own). Same skybeta contract as snowcrashers_nav: we publish ENU
poses/twists to flight_interface, which alone talks to PX4; the pilot arms in Altitude, then
switches to Position on the ground, and that is the go signal.

    WAIT        pose at the current position, 10 Hz (needed for the takeover). On the ground,
                the floor fit gives the camera's height above the floor.
    CLIMB       to cruise_height_m (below the wall top: the camera, pitched 27 deg down, only
                sees things below its own height)
    SETTLE      hover; needs live depth from stereo_depth, or lands in place
    APPROACH    move the target forward at approach_speed_mps until the wall reads within
                stop_distance_m (+ reaction_m) twice in a row. Stale depth: stop advancing
                (depth_timeout_s: land). Nothing within max_forward_m: land in place.
    HOLD_WALL   stop and hold, measure the wall distance
    CLIMB_OVER  climb, holding stop_distance_m from the wall *by depth* (this also cancels a
                VIO frame tilted by the camera mount, which drifts the drone sideways as it
                changes height). wall_height_m > 0: to that + over_margin_m. Otherwise until
                the wall's top edge comes into view below the camera, then to the measured top
                + over_margin_m. Top never seen by max_height_m: wall still in front -> too
                tall, land in front of it; nothing seen at all -> go over at max height.
    OVER        fly forward at that height to land_beyond_wall_m past the wall face (measured by
                depth at the stop; start_to_wall_m if no reading) (pauses and climbs if something
                comes within guard_m)
    ARRIVE / DESCEND / TOUCHDOWN / DONE   as snowcrashers_nav: poses to the 0.3 m floor,
                then a 0.4 m/s downward twist (PX4's land detector needs a commanded
                descent >= 0.275 m/s) until PX4 detects the landing and disarms.

Heights are PX4's own estimate (local_enu in /skybeta/state), the frame PX4 steers in. The
flight interface clamps every pose to 0.3-3.0 m above the *vision* origin (origin_enu), so
targets are compared after that same clamp (on skybeta-03 PX4's height read 0.3-0.45 m
below vision's on the ground).
"""
import math

from overwall_nav.perception import analyze


class Params:
    def __init__(self, **overrides):
        self.cruise_height_m = 0.7
        self.approach_speed_mps = 0.3
        self.approach_accel_mps2 = 0.3
        self.stop_distance_m = 0.5       # wall distance (horizontal, from the camera) to stop at
        self.reaction_m = 0.2            # trigger this much earlier (5 Hz depth, ~0.2 s latency, braking)
        self.stop_readings = 2           # consecutive readings needed to stop
        self.floor_margin_m = 0.25       # also stop if the floor is hidden beyond this much earlier
        self.max_forward_m = 4.57        # 15 ft: no wall by then, land in place
        self.wall_height_m = 0.0         # > 0: known wall height above the floor; 0: measure it
        self.over_margin_m = 0.4         # camera this far above the wall top when crossing
        self.max_height_m = 2.2          # never above this above the start (interface caps at 3.0)
        self.climb_speed_mps = 0.3
        self.keep_gain = 0.6             # 1/s, holding the wall distance by depth while climbing
        self.keep_max_mps = 0.10
        self.keep_window = 7             # readings in the median (~1.4 s at 5 Hz): a cross bar sticking
        self.keep_deadband_m = 0.08      # out of the wall passes by without the drone backing off
        self.land_beyond_wall_m = 1.880  # 6 ft 2 in: land with the drone's centre this far past the wall face
        self.start_to_wall_m = 0.0       # > 0: wall distance expected from the take-off spot (check + fallback)
        self.camera_forward_m = 0.105    # camera ahead of the drone's centre (base_link -> camera TF)
        self.over_speed_mps = 0.35
        self.over_accel_mps2 = 0.3
        self.guard_m = 0.35              # in OVER, something this close: pause and climb
        self.camera_height_m = 0.11      # camera above the floor when landed (carpet floor fit, 2026-11-07)
        self.floor_pitch_tol_deg = 5.0   # only trust a floor fit whose angle matches the mount
        self.min_cell = 40.0             # points a 5 cm depth cell needs to count (perception.py)
        self.depth_stale_s = 0.7
        self.depth_timeout_s = 3.0
        self.descent_fast_mps = 0.5
        self.descent_slow_mps = 0.4
        self.hold_s = 1.5
        self.reach_z_m = 0.12
        self.reach_xy_m = 0.20
        self.arrive_timeout_s = 15.0
        self.xy_gain = 0.8
        self.xy_max_mps = 0.3
        self.floor_m = 0.3               # the interface's min_altitude_m
        self.ceiling_m = 2.5             # the interface's max_altitude_m (set in overwall.launch.py)
        self.net_top_m = 2.896           # 9.5 ft: top of the net, above the floor
        self.net_clearance_m = 0.4       # the drone's top stays this far below it
        self.body_above_m = 0.08         # props above the drone's centre
        self.camera_above_m = 0.03       # camera above the drone's centre
        self.stale_s = 1.0
        self.repeat = True
        for key, value in overrides.items():
            if not hasattr(self, key):
                raise AttributeError(key)
            setattr(self, key, type(getattr(self, key))(value))
        if self.descent_slow_mps < 0.3:
            raise ValueError('descent_slow_mps must be >= 0.3 m/s or PX4 never detects the landing')
        if self.max_height_m <= self.cruise_height_m:
            raise ValueError('max_height_m must be above cruise_height_m')
        if self.net_top_m - self.net_clearance_m - self.body_above_m - 0.2 <= self.cruise_height_m:
            raise ValueError('cruise_height_m leaves no room under net_top_m')


def trapezoid(t, distance, speed, accel):
    """(distance along, done) t s into a move that speeds up and slows down at ``accel``."""
    if distance <= 0:
        return 0.0, True
    v = min(speed, math.sqrt(accel * distance))
    ramp = v / accel
    cruise = (distance - v * ramp) / v
    total = 2 * ramp + cruise
    if t <= 0:
        return 0.0, False
    if t < ramp:
        return 0.5 * accel * t * t, False
    if t < ramp + cruise:
        return 0.5 * v * ramp + v * (t - ramp), False
    if t < total:
        left = total - t
        return distance - 0.5 * accel * left * left, False
    return distance, True


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Mission:
    """step(now, state, odom, front) -> ('pose', xyz) | ('twist', xyz) | None; poses carry hold_yaw.

    state: /skybeta/state dict + 'received'; odom: {'position', 'yaw', 'received'};
    front: the /overwall/front dict from stereo_depth ('cells', 'top_edge_deg', 'floor')
    + 'received', or None.
    """

    FLYING = ('CLIMB', 'SETTLE', 'APPROACH', 'HOLD_WALL', 'CLIMB_OVER', 'OVER', 'ARRIVE',
              'DESCEND', 'TOUCHDOWN')

    def __init__(self, params=None):
        self.p = params or Params()
        self.phase, self.since, self.log = 'WAIT', 0.0, []
        self.start = None
        self.forward = (1.0, 0.0)
        self.target = None
        self.goal = None
        self.land_xy = None
        self.hold_yaw = None             # ENU yaw to hold in every pose while flying (None: none)
        self.origin_z = None
        self.reached_since = None
        self.saw_not_offboard = False
        self.inherited_logged = False
        self.last_step = None
        self.ground_floor = []           # floor-fit camera heights seen while landed
        self.result = {}
        self._reset_phase_vars()

    def _reset_phase_vars(self):
        self.along_target = 0.0
        self.speed = 0.0
        self.over_z = None
        self.over_from = 0.0
        self.over_t = 0.0
        self.blocked_since = None
        self.descend_from = 0.0
        self.floor_since = None
        self.touchdown_at = None
        self.depth_lost_since = None
        self.wall_readings = []
        self.wall_d = None
        self.wall_top = None
        self.wall_along = None           # wall face, metres from the take-off spot along the heading
        self.over_len = 0.0
        self.top_readings = []
        self.keep_readings = []
        self.keep_last = None
        self.floor_seen = False
        self.stop_why = None
        self.close_count = 0
        self.last_front_stamp = None
        self.cam_ground = None

    # -- helpers ---------------------------------------------------------------
    def say(self, text):
        self.log.append(text)

    def enter(self, phase, now, text=None):
        if text:
            self.say(text)
        self.phase, self.since, self.reached_since = phase, now, None
        if phase in ('WAIT', 'DONE', 'ABORTED'):
            self.hold_yaw = None

    @staticmethod
    def _fresh(item, now, limit):
        return item is not None and now - item.get('received', -1e9) <= limit

    def where(self, now, state, odom):
        if self._fresh(state, now, self.p.stale_s) and state.get('local_enu'):
            return tuple(float(v) for v in state['local_enu'])
        if self._fresh(odom, now, self.p.stale_s):
            return tuple(float(v) for v in odom['position'])
        return None

    def clamp_z(self, z):
        """What the flight interface will actually send for a pose height ``z``."""
        if self.origin_z is None:
            return z
        return min(max(z, self.origin_z + self.p.floor_m), self.origin_z + self.p.ceiling_m)

    def _held(self, ok, now):
        if not ok:
            self.reached_since = None
            return False
        if self.reached_since is None:
            self.reached_since = now
        return now - self.reached_since >= self.p.hold_s

    def _along(self, here):
        return (here[0] - self.start[0]) * self.forward[0] + (here[1] - self.start[1]) * self.forward[1]

    def _point(self, along, z):
        return (self.start[0] + along * self.forward[0], self.start[1] + along * self.forward[1], z)

    def height_cap(self):
        """Highest centre height above the start allowed: max_height_m, and the drone's top
        net_clearance_m below the net (heights above the floor: the centre sits
        cam_ground - camera_above_m above it when landed)."""
        centre_on_ground = self.cam_ground - self.p.camera_above_m
        net = self.p.net_top_m - self.p.net_clearance_m - self.p.body_above_m - centre_on_ground
        return min(self.p.max_height_m, net)

    def cam_height(self, here):
        """Camera height above the floor: its height when landed + PX4's climb since the start."""
        return self.cam_ground + (here[2] - self.start[2])

    def descend(self, now, where, text):
        self.land_xy = (float(where[0]), float(where[1]))
        self.descend_from = self.target[2] if self.target else where[2]
        self.floor_since = None
        self.enter('DESCEND', now, text)

    # -- the tick --------------------------------------------------------------
    def step(self, now, state, odom, front):
        p = self.p
        dt = 0.0 if self.last_step is None else max(0.0, min(0.5, now - self.last_step))
        self.last_step = now
        link = self._fresh(state, now, 2.0) and bool(state.get('px4_link'))
        armed = link and bool(state.get('armed'))
        offboard = link and bool(state.get('offboard_active'))
        if link and state.get('origin_enu'):
            self.origin_z = float(state['origin_enu'][2])
        here = self.where(now, state, odom)
        depth_ok = self._fresh(front, now, p.depth_stale_s)
        new_front = depth_ok and front.get('received') != self.last_front_stamp
        if new_front:
            self.last_front_stamp = front.get('received')

        if self.phase in ('DESCEND', 'TOUCHDOWN') and link and not armed:
            self.enter('DONE', now)
            if here is not None:
                self.result['landed_forward_m'] = round(self._along(here), 2)
            self.say('Disarmed. RESULT %s' % self.result)
            return ('pose', here) if here is not None else None
        if self.phase in self.FLYING and not offboard:
            self.enter('ABORTED', now, 'Offboard ended in %s (armed=%s): the pilot has the drone. '
                       'Mission stopped; nothing is published until the next arming.' % (self.phase, armed))
        if self.phase in ('DONE', 'ABORTED'):
            if not armed and now - self.since >= 3.0 and p.repeat:
                self.enter('WAIT', now, 'Disarmed: ready for the next flight (arm in Altitude, then Position).')
                self.saw_not_offboard = False
            return None

        if self.phase == 'WAIT':
            floor = front.get('floor') or {} if depth_ok else {}
            if (link and state.get('landed') and floor.get('height_m') and
                    abs(float(floor.get('pitch_deg', 1e9)) - float(front.get('pitch_deg', 27.0))) <= p.floor_pitch_tol_deg):
                self.ground_floor = (self.ground_floor + [float(floor['height_m'])])[-30:]
            if not offboard:
                self.saw_not_offboard = True
                return ('pose', here) if here is not None else None
            if not self.saw_not_offboard:
                if not self.inherited_logged:
                    self.inherited_logged = True
                    self.say('PX4 reports Offboard, but this node did not see it start: not ours to fly, '
                             'publishing nothing until the mode switch leaves Offboard.')
                return None
            if here is None or odom is None:
                self.enter('ABORTED', now, 'Offboard started but there is no position or heading: not flying.')
                return None
            self.start = here
            # Hold PX4's own heading from here on. With no yaw in the setpoint PX4 1.16 sets its
            # yaw target to the current yaw every loop (PositionControl.cpp: "TODO: better way to
            # disable yaw control"), so nothing holds the heading: on skybeta-03 the takeoff
            # yawed ~30 deg left and the camera then looked off to the side of the path.
            yaw = float(odom['px4_yaw']) if odom.get('px4_yaw') is not None else float(odom['yaw'])
            self.forward = (math.cos(yaw), math.sin(yaw))
            self._reset_phase_vars()
            if self.ground_floor:
                heights = sorted(self.ground_floor)
                self.cam_ground = heights[len(heights) // 2]
                source = 'floor fit'
            else:
                self.cam_ground = p.camera_height_m
                source = 'camera_height_m (no floor fit)'
            self.result = {'start': [round(v, 2) for v in here], 'yaw_deg': round(math.degrees(yaw), 1),
                           'camera_height_m': round(self.cam_ground, 3)}
            self.result['height_cap_m'] = round(self.height_cap(), 2)
            self.hold_yaw = yaw
            self.enter('CLIMB', now, 'Offboard active: start (%.2f, %.2f, %.2f), heading %.0f deg (held), camera %.2f m '
                       'above the floor (%s). Height limit %.2f m (net top %.2f m). Climbing to %.2f m.'
                       % (*here, math.degrees(yaw), self.cam_ground, source, self.height_cap(), p.net_top_m,
                          p.cruise_height_m))

        if here is None:
            if self.phase == 'TOUCHDOWN':
                return ('twist', (0.0, 0.0, -p.descent_slow_mps))
            return ('pose', self.target) if self.target else None
        t = now - self.since
        along = self._along(here)
        height = here[2] - self.start[2]
        self.result['max_height_m'] = round(max(self.result.get('max_height_m', 0.0), height), 2)
        top_z = self.start[2] + self.height_cap()
        if self.origin_z is not None:
            top_z = min(top_z, self.origin_z + p.ceiling_m - 0.1)
        seen = None
        if depth_ok:
            seen = analyze(front.get('cells', []), self.cam_height(here), float(front.get('top_edge_deg', 0.0)),
                           wall_distance=self.wall_d, min_cell=p.min_cell)
        distance = seen['front'] if seen else None

        if self.phase == 'CLIMB':
            cruise = self.start[2] + p.cruise_height_m
            z = min(cruise, self.start[2] + p.floor_m + p.climb_speed_mps * 1.5 * t)
            self.target = (self.start[0], self.start[1], z)
            if z >= cruise - 1e-6 and abs(here[2] - self.clamp_z(cruise)) < p.reach_z_m:
                self.enter('SETTLE', now, 'At %.2f m.' % height)
            elif t > p.cruise_height_m / p.climb_speed_mps + 15.0:
                self.descend(now, here, 'Could not reach cruise height (at %.2f m): landing here.' % height)

        elif self.phase == 'SETTLE':
            if t >= p.hold_s:
                if not depth_ok:
                    self.descend(now, here, 'No depth from stereo_depth: not approaching, landing here.')
                else:
                    self.along_target, self.speed = along, 0.0
                    self.enter('APPROACH', now, 'Front %s. Approaching at %.2f m/s, stopping %.2f m from the wall.'
                               % ('%.2f m' % distance if distance is not None else 'clear',
                                  p.approach_speed_mps, p.stop_distance_m))

        elif self.phase == 'APPROACH':
            if not depth_ok:
                self.depth_lost_since = self.depth_lost_since or now
                self.speed = 0.0                                    # freeze the target
                if now - self.depth_lost_since > p.depth_timeout_s:
                    self.descend(now, here, 'Depth lost for %.0f s: landing here.' % p.depth_timeout_s)
            else:
                self.depth_lost_since = None
                if new_front:
                    trigger = p.stop_distance_m + p.reaction_m
                    free = seen['floor_free']
                    if free is not None and free > trigger + p.floor_margin_m + 0.3:
                        self.floor_seen = True
                    # The floor rules are for a wall stereo cannot see; a seen wall decides by itself.
                    if distance is not None:
                        why = 'wall %.2f m ahead' % distance if distance <= trigger else None
                    elif free is not None and free <= trigger + p.floor_margin_m:
                        why = 'floor hidden beyond %.2f m (untextured wall?)' % free
                    elif self.floor_seen and free is None:
                        why = 'floor ahead vanished (something right in front?)'
                    else:
                        why = None
                    self.close_count = self.close_count + 1 if why else 0
                    self.stop_why = why
                if self.close_count >= p.stop_readings:
                    self.along_target = along
                    if distance is not None:
                        self.result['stop_reading_m'] = round(distance, 2)
                    else:
                        self.result['stopped_on'] = 'floor occlusion'
                    self.enter('HOLD_WALL', now, 'Stopping at %.2f m forward: %s.' % (along, self.stop_why))
                elif self.along_target >= p.max_forward_m:
                    # Wait for the drone itself, not just its target (it lags ~0.2 m behind):
                    # with a short backstop the wall may only read close enough on the way there.
                    if along >= p.max_forward_m - 0.05 or t > 30.0:
                        self.descend(now, here, 'No wall within %.2f m: landing here.' % p.max_forward_m)
                else:
                    self.speed = min(p.approach_speed_mps, self.speed + p.approach_accel_mps2 * dt)
                    self.along_target = min(p.max_forward_m, self.along_target + self.speed * dt)
            if self.phase == 'APPROACH':
                self.target = self._point(self.along_target, self.start[2] + p.cruise_height_m)

        elif self.phase == 'HOLD_WALL':
            self.target = self._point(self.along_target, self.start[2] + p.cruise_height_m)
            if new_front and distance is not None:
                self.wall_readings.append(distance)
            if t >= p.hold_s:
                if self.wall_readings:
                    self.wall_readings.sort()
                    self.wall_d = self.wall_readings[len(self.wall_readings) // 2]
                    self.result['wall_distance_m'] = round(self.wall_d, 2)
                else:
                    self.wall_d = p.stop_distance_m
                # Where the wall face is, measured from the take-off spot along the heading.
                if self.wall_readings:
                    self.wall_along = along + p.camera_forward_m + self.wall_d
                    if p.start_to_wall_m > 0 and abs(self.wall_along - p.start_to_wall_m) > 0.3:
                        self.say('Depth puts the wall %.2f m from the start, the settings say %.2f m: using depth.'
                                 % (self.wall_along, p.start_to_wall_m))
                elif p.start_to_wall_m > 0:
                    self.wall_along = p.start_to_wall_m
                    self.say('No depth reading of the wall face: using start_to_wall_m %.2f m.' % p.start_to_wall_m)
                else:
                    self.wall_along = along + p.camera_forward_m + p.stop_distance_m + p.reaction_m
                self.result['wall_at_forward_m'] = round(along, 2)
                self.result['wall_from_start_m'] = round(self.wall_along, 2)
                if p.wall_height_m > 0:
                    self.over_z = min(top_z, self.start[2] + p.wall_height_m + p.over_margin_m - self.cam_ground)
                self.enter('CLIMB_OVER', now, 'Wall %.2f m ahead. Climbing over%s.' % (
                    self.wall_d, ' to %.2f m (wall_height_m)' % (self.over_z - self.start[2]) if self.over_z else ''))

        elif self.phase == 'CLIMB_OVER':
            if distance is not None and new_front:                    # hold the gap by depth
                # Filtered: the median of the last keep_window readings, a deadband, a slow rate.
                # A bar or ledge sticking out of the wall reads ~10 cm closer for the moment the
                # camera passes it; without this the drone would back off and creep back.
                self.keep_readings = (self.keep_readings + [distance])[-int(p.keep_window):]
                gap = sorted(self.keep_readings)[len(self.keep_readings) // 2]
                error = gap - p.stop_distance_m
                if len(self.keep_readings) >= min(3, int(p.keep_window)) and abs(error) > p.keep_deadband_m:
                    error -= math.copysign(p.keep_deadband_m, error)
                    dt_front = min(0.5, max(0.0, now - (self.keep_last or now)))
                    self.along_target += max(-p.keep_max_mps, min(p.keep_max_mps, p.keep_gain * error)) * dt_front
                self.keep_last = now
            if p.wall_height_m <= 0 and seen and seen['top_visible'] and new_front:
                top = self.cam_height(here) + seen['top_u']             # wall top above the floor
                if self.wall_top is None:
                    self.say('Wall top in view: %.2f m above the floor.' % top)
                self.top_readings.append(top)
                ordered = sorted(self.top_readings)
                self.wall_top = ordered[len(ordered) // 2]               # running median
                self.result['wall_top_m'] = round(self.wall_top, 2)
                self.over_z = min(top_z, self.start[2] + self.wall_top + p.over_margin_m - self.cam_ground)
            goal_z = self.over_z if self.over_z is not None else top_z
            z_now = self.target[2]
            z = min(goal_z, z_now + p.climb_speed_mps * dt) if z_now < goal_z else goal_z
            self.target = self._point(self.along_target, z)
            at_goal = z >= goal_z - 1e-6 and abs(here[2] - self.clamp_z(goal_z)) < p.reach_z_m
            if self.over_z is not None:
                if self._held(at_goal, now):
                    self.over_from, self.over_t = self.along_target, 0.0
                    land_at = self.wall_along + p.land_beyond_wall_m
                    self.over_len = max(0.3, land_at - self.over_from)
                    self.result['land_target_from_start_m'] = round(land_at, 2)
                    self.enter('OVER', now, 'At %.2f m: flying %.2f m forward over the wall, to land %.2f m past it '
                               '(%.2f m from the start).' % (height, self.over_len, p.land_beyond_wall_m, land_at))
            elif at_goal:                                            # max height, top never seen
                if distance is not None:
                    self.descend(now, here, 'Wall still %.2f m ahead at the %.2f m limit: too tall, landing '
                                 'in front of it.' % (distance, self.height_cap()))
                else:
                    self.over_z = goal_z
                    self.say('Wall top never seen by %.2f m and nothing ahead: going over at that height.'
                             % self.height_cap())

        elif self.phase == 'OVER':
            blocked = distance is not None and distance < p.guard_m
            if blocked:
                self.blocked_since = self.blocked_since or now
                self.over_z = min(top_z, self.over_z + p.climb_speed_mps * dt)
                if self.over_z >= top_z - 1e-6 and now - self.blocked_since > 3.0:
                    self.descend(now, here, 'Blocked %.2f m ahead at the height limit: landing here.' % distance)
            else:
                self.blocked_since = None
                self.over_t += dt                                    # progress only while clear
            if self.phase == 'OVER':
                s, done = trapezoid(self.over_t, self.over_len, p.over_speed_mps, p.over_accel_mps2)
                self.target = self._point(self.over_from + s, self.over_z)
                if done:
                    self.goal = self.target
                    self.enter('ARRIVE', now)

        elif self.phase == 'ARRIVE':
            self.target = self.goal
            miss = math.hypot(here[0] - self.goal[0], here[1] - self.goal[1])
            if self._held(miss < p.reach_xy_m, now):
                self.descend(now, self.goal, 'Past the wall (%.2f m forward). Landing.' % along)
            elif t > p.arrive_timeout_s:
                self.descend(now, here, 'Still %.2f m from the landing point: landing here.' % miss)

        if self.phase == 'DESCEND':
            floor = self.clamp_z(self.start[2] + p.floor_m)
            z = max(floor, self.descend_from - p.descent_fast_mps * (now - self.since))
            self.target = (self.land_xy[0], self.land_xy[1], z)
            if z > floor + 1e-6:
                self.floor_since = None
            elif self.floor_since is None:
                self.floor_since = now
            if self.floor_since is not None and (here[2] < floor + 0.15 or now - self.floor_since > 3.0):
                self.enter('TOUCHDOWN', now, 'At %.2f m: final descent at %.2f m/s.' % (height, p.descent_slow_mps))

        if self.phase == 'TOUCHDOWN':
            landed = bool(link and state.get('landed'))
            if landed and self.touchdown_at is None:
                self.touchdown_at = now
                self.say('PX4 reports landed (%.2f m forward): waiting for it to disarm.' % along)
            if now - self.since > 20.0 and 'slow_disarm' not in self.result:
                self.result['slow_disarm'] = True
                self.say('Still armed 20 s into the final descent: pilot, disarm once it is on the floor.')
            if landed:
                return ('twist', (0.0, 0.0, -p.descent_slow_mps))
            vx, vy = (p.xy_gain * (self.land_xy[k] - here[k]) for k in range(2))
            scale = min(1.0, p.xy_max_mps / max(1e-9, math.hypot(vx, vy)))
            return ('twist', (vx * scale, vy * scale, -p.descent_slow_mps))

        return ('pose', self.target)
