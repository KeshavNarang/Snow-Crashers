"""mission.py against a model of the drone side: the real skybeta AutoOffboard rules and
SafetyEnvelope (imported unchanged), the flight interface's setpoint handling, a simple PX4
(position / velocity Offboard, takeoff delay, the v1.16 land-detector conditions,
COM_DISARM_LAND / COM_DISARM_PRFLT) and the pilot's arm-then-Position sequence.

    python -m pytest team_ws/src/snowcrashers_nav/test        (or python -m unittest)
"""
import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
try:
    from skybeta_flight_interface.auto_offboard import AutoOffboard, Inputs
    from skybeta_flight_interface.envelope import SafetyEnvelope
except ImportError:      # on a laptop: the drone's source from the repo's backup of skybeta-03
    backup = sorted((HERE.parents[3] / 'jetson-backups').glob('skybeta-03-*/skybeta/src/skybeta_flight_interface'))
    sys.path.insert(0, str(backup[-1]))
    from skybeta_flight_interface.auto_offboard import AutoOffboard, Inputs
    from skybeta_flight_interface.envelope import SafetyEnvelope

from snowcrashers_nav.mission import Mission, Params, trapezoid   # noqa: E402

ALTCTL, POSCTL, OFFBOARD = 1, 2, 14
DT = 0.02


class Drone:
    """Interface + PX4 + vehicle + pilot. Positions ENU; PX4's local frame == odom here."""

    def __init__(self, yaw_deg=30.0, start=(0.4, -0.2, 0.0), arm_at=4.0, arm_mode=ALTCTL, switch_at=6.0,
                 takeover_at=None, odometry=True, params=None, airborne_offboard=False, node_dies_at=None,
                 local_offset_z=0.0):
        self.node_dies_at = node_dies_at
        self.offset = (0.0, 0.0, local_offset_z)     # PX4's local height minus the vision height
        self.mission = Mission(Params(**(params or {})))
        self.auto = AutoOffboard(True, 1.0)
        self.envelope = SafetyEnvelope()
        self.t = 0.0
        self.p, self.v = list(start), [0.0, 0.0, 0.0]
        self.origin = tuple(start)
        self.yaw = math.radians(yaw_deg)
        self.arm_at, self.arm_mode, self.switch_at, self.takeover_at = arm_at, arm_mode, switch_at, takeover_at
        self.odometry = odometry
        self.armed, self.nav, self.landed = False, ALTCTL, True
        self.armed_at = self.offboard_at = self.took_off_at = self.contact_since = self.landed_at = None
        self.mode, self.target_pos, self.target_vel, self.hold_pos, self.last_sp = 'hold', None, None, None, None
        self.state = self.odom = None
        self.published, self.events, self.trace = [], [], []
        self.touchdown_speed = None
        if airborne_offboard:       # the node (re)starts with the drone already flying in Offboard
            self.armed, self.nav, self.landed, self.p[2] = True, OFFBOARD, False, 1.0
            self.armed_at = self.took_off_at = self.offboard_at = 0.0
            self.mode, self.target_pos = 'position', tuple(self.p)

    # -- flight interface --------------------------------------------------
    def take(self, command):
        if command is None:
            return
        kind, xyz = command
        self.published.append((round(self.t, 2), kind))
        self.last_sp = self.t
        if kind == 'pose':
            self.target_pos = self.envelope.clamp_position(xyz, self.origin).value
            self.hold_pos, self.mode = self.target_pos, 'position'
        else:
            self.target_vel, self.mode = self.envelope.clamp_velocity(xyz).value, 'velocity'

    def interface_tick(self):
        if self.mode == 'velocity' and self.t - self.last_sp > 1.0:
            self.mode = 'hold'
        age = None if self.last_sp is None else self.t - self.last_sp
        request, message = self.auto.step(Inputs(
            now_s=self.t, link=True, armed=self.armed, in_position=self.nav == POSCTL, landed=self.landed,
            offboard=self.nav == OFFBOARD, position_valid=True, setpoint_age_s=age, frozen=False))
        if message:
            self.events.append((round(self.t, 2), 'interface: ' + message))
        if request and self.armed and self.nav == POSCTL:
            self.nav, self.offboard_at = OFFBOARD, self.t

    # -- PX4 + vehicle ----------------------------------------------------------
    def px4_step(self):
        t = self.t
        if not self.armed and t >= self.arm_at and self.armed_at is None:
            self.armed, self.nav, self.armed_at = True, self.arm_mode, t
        if self.armed and self.switch_at is not None and t >= self.switch_at and self.nav == ALTCTL \
                and self.offboard_at is None:
            self.nav = POSCTL
        if self.takeover_at is not None and t >= self.takeover_at and self.nav == OFFBOARD:
            self.nav = ALTCTL                       # pilot flips the mode switch
        want = [0.0, 0.0, 0.0]
        if self.armed and self.nav == OFFBOARD:
            if self.mode == 'velocity':
                want = list(self.target_vel)
            elif self.hold_pos is not None:
                gain = (0.95, 0.95, 1.0)           # PX4 steers its own local estimate to the target
                want = [gain[k] * (self.hold_pos[k] - self.p[k] - self.offset[k]) for k in range(3)]
                h = math.hypot(want[0], want[1])
                if h > 12.0:
                    want[0], want[1] = want[0] * 12.0 / h, want[1] * 12.0 / h
                want[2] = max(-1.5, min(3.0, want[2]))
            if self.took_off_at is None and t - self.offboard_at < 1.5:
                want = [0.0, 0.0, 0.0]              # PX4's takeoff thrust ramp
        on_ground = self.p[2] <= 1e-6
        for k in range(3):
            self.v[k] += (want[k] - self.v[k]) * min(1.0, DT / 0.25)
        if on_ground and self.v[2] < 0:
            self.v = [0.0, 0.0, 0.0] if self.landed or self.took_off_at is None else [self.v[0] * 0.5, self.v[1] * 0.5, 0.0]
        for k in range(3):
            self.p[k] += self.v[k] * DT
        if self.p[2] < 0:
            if self.touchdown_speed is None and self.took_off_at is not None:
                self.touchdown_speed = -self.v[2]
            self.p[2], self.v[2] = 0.0, 0.0
        if self.p[2] > 0.05 and self.took_off_at is None:
            self.took_off_at, self.landed = t, False
            self.events.append((round(t, 2), 'took off'))
        # PX4 v1.16 land detector, simplified: on the floor, commanded descent >= 1.1 x 0.25 m/s
        # (Offboard velocity mode), not moving -> ground contact; held -> landed.
        descending = (self.nav == OFFBOARD and self.mode == 'velocity' and self.target_vel[2] <= -0.275)
        if self.armed and self.took_off_at is not None and not self.landed:
            if self.p[2] <= 1e-6 and descending:
                self.contact_since = t if self.contact_since is None else self.contact_since
                if t - self.contact_since >= 2.0:
                    self.landed, self.landed_at = True, t
                    self.events.append((round(t, 2), 'landing detected'))
            else:
                self.contact_since = None
        if self.armed and self.landed and self.landed_at is not None and t - self.landed_at >= 2.0:
            self.armed = False                       # COM_DISARM_LAND
            self.events.append((round(t, 2), 'auto-disarm: landed'))
        if self.armed and self.took_off_at is None and t - self.armed_at >= 10.0:
            self.armed = False                       # COM_DISARM_PRFLT
            self.events.append((round(t, 2), 'auto-disarm: no takeoff within 10 s'))

    # -- run ----------------------------------------------------------------------
    def run(self, duration=60.0):
        steps = int(round(duration / DT))
        for i in range(steps):
            self.t = i * DT
            if i % 10 == 0:          # /skybeta/state at 5 Hz
                self.state = {'px4_link': True, 'armed': self.armed, 'landed': self.landed,
                              'offboard_active': self.nav == OFFBOARD,
                              'local_enu': [self.p[k] + self.offset[k] for k in range(3)],
                              'received': self.t}
            if self.odometry and i % 2 == 0:
                self.odom = {'position': tuple(self.p), 'yaw': self.yaw, 'received': self.t}
            if i % 5 == 0:           # node and interface at 10 Hz
                if self.node_dies_at is None or self.t < self.node_dies_at:
                    self.take(self.mission.step(self.t, self.state, self.odom))
                for line in self.mission.log:
                    self.events.append((round(self.t, 2), '[%s] %s' % (self.mission.phase, line)))
                self.mission.log.clear()
                self.interface_tick()
            self.px4_step()
            self.trace.append((self.t, tuple(self.p), self.mission.phase))
        return self

    def forward_and_side(self):
        dx, dy = self.p[0] - self.origin[0], self.p[1] - self.origin[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return dx * c + dy * s, -dx * s + dy * c


class MissionTest(unittest.TestCase):
    def test_full_flight(self):
        d = Drone().run()
        log = '\n'.join('%6.2f %s' % e for e in d.events)
        self.assertIsNotNone(d.offboard_at, log)
        self.assertLess(d.took_off_at - d.armed_at, 10.0, log)         # beat COM_DISARM_PRFLT
        top = max(p[2] for _, p, _ in d.trace)
        self.assertGreater(top, 1.85, log)
        self.assertLess(top, 2.2, log)
        forward, side = d.forward_and_side()
        self.assertAlmostEqual(forward, 2.0, delta=0.2, msg=log)
        self.assertLess(abs(side), 0.15, log)
        self.assertFalse(d.armed, log)
        self.assertTrue(d.landed, log)
        self.assertLess(d.touchdown_speed, 0.45, log)
        self.assertIn('auto-disarm: landed', log)
        self.assertEqual(d.mission.phase, 'WAIT', log)                  # ready for the next arming
        self.assertIn('RESULT', log)

    def test_setpoints_flow_before_arming(self):
        d = Drone(arm_at=1e9).run(10.0)
        times = [t for t, kind in d.published if kind == 'pose']
        gaps = [b - a for a, b in zip(times, times[1:])]
        self.assertGreater(len(times), 80)
        self.assertLess(max(gaps), 0.2)

    def test_pilot_takeover_stops_the_mission(self):
        d = Drone(takeover_at=10.0).run(20.0)
        self.assertEqual(d.mission.phase, 'ABORTED')
        after = [t for t, _ in d.published if t > 10.3]
        self.assertEqual(after, [])

    def test_arming_on_position_gives_no_autonomy(self):
        d = Drone(arm_mode=POSCTL).run(20.0)
        self.assertIsNone(d.offboard_at)
        self.assertIsNone(d.took_off_at)
        self.assertEqual(d.mission.phase, 'WAIT')
        self.assertIn('no takeoff within 10 s', ' '.join(e for _, e in d.events))

    def test_never_flies_an_offboard_it_did_not_see_start(self):
        d = Drone(airborne_offboard=True, arm_at=0.0).run(5.0)
        self.assertEqual(d.mission.phase, 'WAIT')
        self.assertEqual(d.published, [])

    def test_no_heading_no_flight(self):
        d = Drone(odometry=False).run(20.0)
        self.assertEqual(d.mission.phase, 'ABORTED')
        self.assertLess(max(p[2] for _, p, _ in d.trace), 0.5)

    def test_slow_switch_still_takes_off_in_time(self):
        d = Drone(switch_at=10.5).run(60.0)     # switch 6.5 s after arming
        self.assertLess(d.took_off_at - d.armed_at, 10.0)
        self.assertFalse(d.armed)

    def test_node_dying_in_the_descent_leaves_it_low(self):
        # The interface falls back to the last pose when twists stop: it must not be the far point.
        full = Drone().run()
        descend = next(t for t, _, phase in full.trace if phase == 'DESCEND')
        touchdown = next(t for t, _, phase in full.trace if phase == 'TOUCHDOWN')
        for dies in (descend + 1.0, touchdown + 0.3):
            d = Drone(node_dies_at=dies).run(dies + 15.0)
            at_death = next(p[2] for t, p, _ in d.trace if t >= dies)
            later = max(p[2] for t, p, _ in d.trace if t >= dies)
            self.assertLess(later, max(at_death, 0.3) + 0.1, 'died at %.1f s' % dies)

    def test_height_offset_between_px4_and_vision_still_lands(self):
        # PX4's height (rangefinder) reads 0.25 m below the vision height: the interface's
        # 0.3 m floor (on vision) then holds the drone above where we expect the floor.
        for offset in (-0.25, 0.15):
            d = Drone(local_offset_z=offset).run(70.0)
            log = '\n'.join('%6.2f %s' % e for e in d.events)
            self.assertFalse(d.armed, log)
            self.assertTrue(d.landed, log)
            self.assertIn('RESULT', log)

    def test_descent_floor_guard(self):
        with self.assertRaises(ValueError):
            Params(descent_slow_mps=0.2)

    def test_trapezoid(self):
        s, done = trapezoid(100.0, 2.0, 0.4, 0.3)
        self.assertTrue(done)
        self.assertAlmostEqual(s, 2.0)
        samples = [trapezoid(i * 0.05, 2.0, 0.4, 0.3)[0] for i in range(200)]
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(samples, samples[1:])))
        self.assertLessEqual(max(b - a for a, b in zip(samples, samples[1:])) / 0.05, 0.4 + 1e-9)


if __name__ == '__main__':
    unittest.main()
