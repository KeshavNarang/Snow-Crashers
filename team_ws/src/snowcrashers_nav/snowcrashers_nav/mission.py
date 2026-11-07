"""Up 2 m, forward 2 m, land: the mission logic, with no ROS in it (tested on its own).

How a flight works on a skybeta drone (skybeta 0.3.3, read on skybeta-03):

1. The stack runs ``flight_interface`` (launched by our competitor.launch.py). It is the only
   thing that talks to PX4. We publish ENU targets on /skybeta/setpoint/pose (a position to
   hold) or /skybeta/setpoint/twist (a velocity, which expires 1 s after the last message).
2. The safety pilot arms on RC with the mode switch on Altitude, then switches to Position on
   the ground. If we have published a setpoint in the last second, the interface asks PX4 for
   Offboard, once per arming. The Jetson cannot arm.
3. From then on our targets fly the drone. The pilot can take it back at any time (sticks,
   mode switch, kill switch); Offboard does not come back until disarm.

Our side of it:

WAIT     publish a pose at the current position, 10 Hz (the interface needs a fresh setpoint
         at the moment of the switch). When Offboard starts, take the start pose.
CLIMB    raise the pose target from 0.3 m (the interface floors every pose there, so the
         takeoff starts at once: PX4 disarms if it has not left the ground 10 s after arming)
         to HEIGHT at CLIMB_SPEED.
SETTLE   hold at the top.
FORWARD  move the pose target FORWARD metres along the nose direction at the start, speeding
         up and slowing down gently (a target that jumps 2 m makes PX4 fly at ~2 m/s:
         MPC_XY_VEL_MAX is 12 m/s on these airframes).
ARRIVE   hold until the drone is there.
DESCEND  lower the pose target to the interface's 0.3 m floor (a pose is also what the
         interface falls back to if this node dies, so it should be low and over the spot).
TOUCHDOWN poses cannot go below 0.3 m, so the last part is twists: straight down at
         DESCENT_SLOW, holding x/y with a small correction. At least 0.3 m/s: PX4 1.16's land
         detector only counts a commanded descent of >= 1.1 x LNDMC_Z_VEL_MAX (0.25 -> 0.275
         m/s). On the floor PX4 detects the landing and disarms after COM_DISARM_LAND (2 s).
DONE     one pose at the landing spot, then nothing (no live setpoint: the interface will
         not take off again on its own); wait for the next arming.

Positions come from PX4's own estimate in /skybeta/state (local_enu), which is what PX4
steers to; the heading comes from cuVSLAM odometry. Both are in the same ENU frame that
the interface converts to PX4's NED.
"""
import math

NAN = float('nan')


class Params:
    def __init__(self, **overrides):
        self.height_m = 2.0
        self.forward_m = 2.0
        self.climb_speed_mps = 0.5
        self.forward_speed_mps = 0.4
        self.forward_accel_mps2 = 0.3
        self.descent_fast_mps = 0.5      # the pose target, down to the 0.3 m floor
        self.descent_slow_mps = 0.4      # twists for the last 0.3 m, and on the floor until disarm
        self.hold_s = 1.5                # settle time at the top and at the far point
        self.reach_z_m = 0.15
        self.reach_xy_m = 0.20
        self.arrive_timeout_s = 15.0
        self.xy_gain = 0.8               # 1/s, x/y correction while descending
        self.xy_max_mps = 0.3
        self.floor_m = 0.3               # the interface's min_altitude_m
        self.stale_s = 1.0               # position older than this: do not advance
        self.repeat = True               # after landing, the next arm + Position flies again
        for key, value in overrides.items():
            if not hasattr(self, key):
                raise AttributeError(key)
            setattr(self, key, type(getattr(self, key))(value))
        if self.descent_slow_mps < 0.3:
            raise ValueError('descent_slow_mps must be >= 0.3 m/s or PX4 never detects the landing')


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
    """step(now, state, odom) -> ('pose', xyz) | ('twist', xyz) | None, once per tick.

    state: the latest /skybeta/state dict plus 'received' (s), or None.
    odom:  {'position': xyz, 'yaw': rad, 'received': s} from cuVSLAM, or None.
    Messages for the log accumulate in self.log (the node prints and clears them).
    """

    FLYING = ('CLIMB', 'SETTLE', 'FORWARD', 'ARRIVE', 'DESCEND', 'TOUCHDOWN')

    def __init__(self, params=None):
        self.p = params or Params()
        self.phase = 'WAIT'
        self.log = []
        self.since = 0.0
        self.start = None
        self.yaw = 0.0
        self.forward = (1.0, 0.0)
        self.top = 0.0
        self.goal = None
        self.land_xy = None
        self.target = None
        self.reached_since = None
        self.touchdown_at = None
        self.saw_not_offboard = False     # Offboard must start while we watch, never inherited
        self.inherited_logged = False
        self.descend_from = 0.0
        self.floor_since = None
        self.last_landed = None
        self.result = {}

    # -- helpers ---------------------------------------------------------------
    def say(self, text):
        self.log.append(text)

    def enter(self, phase, now, text=None):
        if text:
            self.say(text)
        self.phase, self.since = phase, now
        self.reached_since = None

    @staticmethod
    def _fresh(item, now, limit):
        return item is not None and now - item.get('received', -1e9) <= limit

    def where(self, now, state, odom):
        """Current ENU position: PX4's estimate if fresh, else cuVSLAM's, else None."""
        if self._fresh(state, now, self.p.stale_s) and state.get('local_enu'):
            return tuple(float(v) for v in state['local_enu'])
        if self._fresh(odom, now, self.p.stale_s):
            return tuple(float(v) for v in odom['position'])
        return None

    def _held(self, ok, now):
        """True once ``ok`` has held for hold_s."""
        if not ok:
            self.reached_since = None
            return False
        if self.reached_since is None:
            self.reached_since = now
        return now - self.reached_since >= self.p.hold_s

    # -- the tick --------------------------------------------------------------
    def step(self, now, state, odom):
        p = self.p
        link = self._fresh(state, now, 2.0) and bool(state.get('px4_link'))
        armed = link and bool(state.get('armed'))
        offboard = link and bool(state.get('offboard_active'))
        if link:
            self.last_landed = state.get('landed')
        here = self.where(now, state, odom)

        if self.phase in ('DESCEND', 'TOUCHDOWN') and link and not armed:
            # PX4 disarmed after detecting the landing (it may also leave Offboard as it does).
            self.enter('DONE', now)
            if here is None:
                self.say('Disarmed. RESULT %s' % self.result)
                return None
            self.result['landed_forward_m'] = round(self._along(here), 2)
            self.say('Disarmed. RESULT %s' % self.result)
            # One pose at the landing spot, so the interface's fallback target is no longer
            # the far point in the air. One message is not a live stream: no takeover follows.
            return ('pose', here)
        if self.phase in self.FLYING and not offboard:
            self.enter('ABORTED', now, 'Offboard ended in %s (armed=%s): the pilot has the drone. '
                       'Mission stopped; nothing is published until the next arming.' % (self.phase, armed))
        if self.phase in ('DONE', 'ABORTED'):
            if not armed and now - self.since >= 3.0 and p.repeat:
                self.enter('WAIT', now, 'Disarmed: ready for the next flight (arm in Altitude, then Position).')
                self.saw_not_offboard = False
            return None

        if self.phase == 'WAIT':
            if not offboard:
                self.saw_not_offboard = True
                # A fresh setpoint at the current position: what lets the interface take over.
                return ('pose', here) if here is not None else None
            if not self.saw_not_offboard:
                if not self.inherited_logged:
                    self.inherited_logged = True
                    self.say('PX4 reports Offboard, but this node did not see it start (node started in '
                             'flight, or PX4 still shows Offboard after a disarm): not ours to fly, '
                             'publishing nothing until the mode switch leaves Offboard.')
                return None
            if here is None or odom is None:
                self.enter('ABORTED', now, 'Offboard started but there is no position or heading: not flying.')
                return None
            self.start = here
            self.yaw = float(odom['yaw'])
            self.forward = (math.cos(self.yaw), math.sin(self.yaw))
            self.top = self.start[2] + p.height_m
            self.goal = (self.start[0] + p.forward_m * self.forward[0],
                         self.start[1] + p.forward_m * self.forward[1], self.top)
            self.result = {'start': [round(v, 2) for v in self.start], 'yaw_deg': round(math.degrees(self.yaw), 1)}
            self.enter('CLIMB', now, 'Offboard active: start (%.2f, %.2f, %.2f), heading %.0f deg. Climbing to %.1f m.'
                       % (*self.start, math.degrees(self.yaw), p.height_m))

        if here is None:
            # No position: do not advance. Keep the last target (PX4 also falls back without VIO).
            if self.phase == 'TOUCHDOWN':
                return ('twist', (0.0, 0.0, -p.descent_slow_mps))
            return ('pose', self.target) if self.target else None
        height = here[2] - self.start[2]
        self.result['max_height_m'] = round(max(self.result.get('max_height_m', 0.0), height), 2)
        along = self._along(here)
        self.result['max_forward_m'] = round(max(self.result.get('max_forward_m', 0.0), along), 2)
        t = now - self.since

        if self.phase == 'CLIMB':
            z = self.start[2] + min(p.height_m, p.floor_m + p.climb_speed_mps * t)
            self.target = (self.start[0], self.start[1], z)
            if z >= self.top - 1e-6 and abs(here[2] - self.top) < p.reach_z_m:
                self.enter('SETTLE', now, 'At %.2f m.' % height)
            elif t > p.height_m / p.climb_speed_mps + 15.0:
                self.descend(now, here, 'Could not reach %.1f m (at %.2f m): landing here.' % (p.height_m, height))
        elif self.phase == 'SETTLE':
            if t >= p.hold_s:
                self.enter('FORWARD', now, 'Flying %.1f m forward.' % p.forward_m)
        elif self.phase == 'FORWARD':
            s, done = trapezoid(t, p.forward_m, p.forward_speed_mps, p.forward_accel_mps2)
            self.target = (self.start[0] + s * self.forward[0], self.start[1] + s * self.forward[1], self.top)
            if done:
                self.enter('ARRIVE', now)
        elif self.phase == 'ARRIVE':
            self.target = self.goal
            miss = math.hypot(here[0] - self.goal[0], here[1] - self.goal[1])
            if self._held(miss < p.reach_xy_m and abs(here[2] - self.top) < 0.25, now):
                self.descend(now, self.goal, 'Arrived %.2f m ahead (%.2f m from the goal). Landing.' % (along, miss))
            elif t > p.arrive_timeout_s:
                self.descend(now, here, 'Still %.2f m from the goal after %.0f s: landing here.'
                             % (miss, p.arrive_timeout_s))

        if self.phase == 'DESCEND':
            # Poses down to the floor: if this node died now, the interface would keep holding
            # the last pose (low, over the landing spot), not climb back to the far point.
            floor = self.start[2] + p.floor_m
            z = max(floor, self.descend_from - p.descent_fast_mps * (now - self.since))
            self.target = (self.land_xy[0], self.land_xy[1], z)
            if z > floor + 1e-6:
                self.floor_since = None
            elif self.floor_since is None:
                self.floor_since = now
            # Normally the drone reads ~0.3 m here. If PX4's height and the vision height
            # disagree, the interface's floor (measured on vision) can hold it a little higher:
            # go on to the final descent anyway once the target has sat on the floor 3 s.
            if self.floor_since is not None and (here[2] < floor + 0.15 or now - self.floor_since > 3.0):
                self.enter('TOUCHDOWN', now, 'At %.2f m: final descent at %.2f m/s.' % (height, p.descent_slow_mps))

        if self.phase == 'TOUCHDOWN':
            t = now - self.since
            landed = bool(link and state.get('landed'))
            if landed and self.touchdown_at is None:
                self.touchdown_at = now
                self.say('PX4 reports landed (%.2f m ahead of the start): waiting for it to disarm.' % along)
            if t > 20.0 and 'slow_disarm' not in self.result:
                self.result['slow_disarm'] = True
                self.say('Still armed %.0f s into the final descent: pilot, disarm once it is on the floor.' % t)
            # Twists, because only a commanded descent lets PX4's land detector see the landing.
            vz = -p.descent_slow_mps
            if landed:
                return ('twist', (0.0, 0.0, vz))
            vx, vy = (p.xy_gain * (self.land_xy[k] - here[k]) for k in range(2))
            scale = min(1.0, p.xy_max_mps / max(1e-9, math.hypot(vx, vy)))
            return ('twist', (vx * scale, vy * scale, vz))

        return ('pose', self.target)

    def descend(self, now, where, text):
        """Land at ``where`` (x, y): down with poses to the floor, then twists to touchdown."""
        self.land_xy = (float(where[0]), float(where[1]))
        self.descend_from = self.target[2] if self.target else self.top
        self.floor_since = None
        self.enter('DESCEND', now, text)

    def _along(self, here):
        return (here[0] - self.start[0]) * self.forward[0] + (here[1] - self.start[1]) * self.forward[1]
