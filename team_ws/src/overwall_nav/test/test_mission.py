"""mission.py against a model of the drone, its camera, and a wall.

Drone side: the real skybeta AutoOffboard rules and SafetyEnvelope (imported unchanged), the
flight interface's setpoint handling, a simple PX4 (position / velocity Offboard, takeoff delay,
the v1.16 land-detector conditions, COM_DISARM_LAND / _PRFLT) and the pilot's
arm-then-Position.

Camera: what stereo_depth would publish. The rig's own pixel grid (every 4th pixel here, so a
cell needs a quarter of the points: min_cell 10) is traced, with the 27 deg down mount, against
the floor and the wall box; ranges get stereo noise (sigma = Z^2 / (f B) * 0.2 px); scattered
false matches are added in the top rows; then the points are binned with perception.bin_points.

Colour camera (only with ``pad=``): what pad_detect would publish. The mat's four corners are
cast into skybeta-03's colour camera (640x400, fx 489.6, 27 deg down); when all four are in the
image and none is hidden behind the wall, the mat is reported (its centre per metre of camera
height, with 2 cm of noise), as pad.PadFinder does. The heading follows the mission's yaw
setpoint at up to 60 deg/s (PX4's MPC_YAWRAUTO_MAX default is 45-60).

Also modelled: a VIO frame tilted by the camera mount (the stack's TF says the camera is level).
PX4 then sees sin(tilt) metres of false forward/backward motion per metre of climb.
"""
from collections import deque
import math
from pathlib import Path
import random
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
try:
    from skybeta_flight_interface.auto_offboard import AutoOffboard, Inputs
    from skybeta_flight_interface.envelope import SafetyEnvelope
except ImportError:      # on a laptop: the drone's source from the repo's backup of skybeta-03
    backup = sorted((HERE.parents[3] / 'jetson-backups').glob('skybeta-03-*/skybeta/src/skybeta_flight_interface'))
    sys.path.insert(0, str(backup[-1]))
    from skybeta_flight_interface.auto_offboard import AutoOffboard, Inputs
    from skybeta_flight_interface.envelope import SafetyEnvelope

from overwall_nav.mission import Mission, Params        # noqa: E402
from overwall_nav.perception import bin_points          # noqa: E402
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, mount_matrix   # noqa: E402
import synthetic                                         # noqa: E402

ALTCTL, POSCTL, OFFBOARD = 1, 2, 14
DT = 0.02
PITCH = 27.0
CAM_FORWARD, CAM_GROUND = 0.105, 0.14          # camera ahead of the centre; above the floor when landed
BODY_HALF_LENGTH, BODY_BELOW, BODY_ABOVE = 0.25, 0.15, 0.08   # relative to the camera height... see body()

_R, _T = extrinsics_from_tf(synthetic.T_LR, synthetic.Q_LR)
RIG = StereoRig(synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, _R, _T, step=4)
_RAYS = np.stack([(RIG.grid_u.ravel() - RIG.cx) / RIG.f, (RIG.grid_v.ravel() - RIG.cy) / RIG.f,
                  np.ones(RIG.grid_u.size)], axis=1) @ RIG.r1
_RAYS /= np.linalg.norm(_RAYS, axis=1, keepdims=True)
LEVEL_RAYS = _RAYS @ mount_matrix(PITCH).T                     # (F, L, U) unit rays
TOP_EDGE = RIG.top_elevation_deg(PITCH)


class World:
    """A drone, its interface, PX4, the pilot, its camera, and one wall ``wall_at`` m ahead."""

    def __init__(self, wall_at=2.5, wall_height=1.0, wall_thickness=0.1, wall_textured=True,
                 background_at=3.0, yaw_deg=20.0, start=(0.3, -0.2, 0.0), arm_at=4.0, switch_at=6.0,
                 takeover_at=None, depth=True, depth_dropout=None, local_offset_z=0.0, vio_tilt_deg=0.0,
                 spurious=60, params=None, seed=1, floor_fit=None, wall_bottom=0.0, crossbar=None,
                 pad=None, pad_feed=None, remote=None):
        self.remote_fn = remote                 # operator: f(t, phase, world) -> controller message or None
        self.remote_msg = None
        self.pad = pad                          # (along, left, yaw_deg) of the mat from the start, or None
        self.pad_feed = pad is not None if pad_feed is None else pad_feed   # pad_detect running
        self.pad_msg = None
        self.crossbar = crossbar                # (bottom, top, protrusion) m: a bar standing out of the wall
        self.wall_bottom = wall_bottom          # the wall face starts this high: a hole below it
        self.floor_fit = floor_fit
        params = dict({'min_cell': 10.0}, **(params or {}))
        self.mission = Mission(Params(**params))
        self.auto, self.envelope = AutoOffboard(True, 1.0), SafetyEnvelope()
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        self.t, self.p, self.v = 0.0, list(start), [0.0, 0.0, 0.0]
        self.origin = tuple(start)
        self.yaw = math.radians(yaw_deg)
        self.heading = self.yaw                 # the drone's real heading (follows hold_yaw)
        self.f = (math.cos(self.yaw), math.sin(self.yaw))
        self.wall = (wall_at, wall_thickness, wall_height) if wall_at is not None else None
        self.wall_textured, self.background_at, self.spurious = wall_textured, background_at, spurious
        self.depth, self.depth_dropout = depth, depth_dropout
        self.arm_at, self.switch_at, self.takeover_at = arm_at, switch_at, takeover_at
        self.offset_z = local_offset_z
        self.tilt = math.sin(math.radians(vio_tilt_deg))
        self.armed, self.nav, self.landed = False, ALTCTL, True
        self.armed_at = self.offboard_at = self.took_off_at = self.contact_since = self.landed_at = None
        self.mode, self.target_vel, self.hold_pos, self.last_sp = 'hold', None, None, None
        self.state = self.odom = self.front = None
        self.history = deque(maxlen=10)
        self.published, self.events, self.trace = [], [], []
        self.contacts = 0
        self.held = []
        self.climb_alongs = []
        self.stop_gap = None
        self.max_gap_error = 0.0
        self.min_gap = None

    def along(self, p=None):
        p = p or self.p
        return (p[0] - self.origin[0]) * self.f[0] + (p[1] - self.origin[1]) * self.f[1]

    def reported(self, p):
        """PX4's / VIO's view of position p: a tilted VIO frame shifts it along the heading."""
        shift = -self.tilt * (p[2] - self.origin[2])
        return [p[0] + shift * self.f[0], p[1] + shift * self.f[1], p[2] + self.offset_z]

    # -- camera ------------------------------------------------------------------
    def cells(self, p):
        cam_f, cam_u = self.along(p) + CAM_FORWARD, p[2] + CAM_GROUND
        d = LEVEL_RAYS
        df, du = d[:, 0], d[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(du < 0, -cam_u / du, np.inf)                        # floor
            if self.background_at is not None and self.wall:
                t = np.minimum(t, np.where(df > 0, (self.wall[0] + self.background_at - cam_f) / df, np.inf))
            if self.wall:
                near, thick, height = self.wall
                t_face = np.where(df > 0, (near - cam_f) / df, np.inf)
                hit_u = cam_u + t_face * du
                t_face = np.where((hit_u >= self.wall_bottom) & (hit_u <= height), t_face, np.inf)
                t_top = np.where(du < 0, (height - cam_u) / du, np.inf)
                hit_f = cam_f + t_top * df
                t_top = np.where((hit_f >= near) & (hit_f <= near + thick), t_top, np.inf)
                wall_t = np.minimum(t_face, t_top)
                if self.crossbar:
                    lo, hi, out = self.crossbar
                    t_bar = np.where(df > 0, (near - out - cam_f) / df, np.inf)
                    bar_u = cam_u + t_bar * du
                    wall_t = np.minimum(wall_t, np.where((bar_u >= lo) & (bar_u <= hi), t_bar, np.inf))
                wall_hit = wall_t < t
                t = np.minimum(t, wall_t)
                if not self.wall_textured:
                    t = np.where(wall_hit, np.inf, t)                        # no match on a blank wall
        ok = np.isfinite(t) & (t < 6.0)
        t = t[ok]
        t = t + self.np_rng.normal(0.0, 1.0, t.size) * t * t / (RIG.f * RIG.baseline) * 0.2
        pts = d[ok] * t[:, None]
        if self.spurious:                                                    # false matches, top rows
            top = d[d[:, 2] > math.sin(math.radians(TOP_EDGE - 3.0))]
            pick = top[self.np_rng.integers(0, len(top), self.spurious)]
            pts = np.vstack([pts, pick * self.np_rng.uniform(0.25, 0.45, (self.spurious, 1))])
        return bin_points(pts, 0.4)

    def pad_view(self, p):
        """pad_detect's message for a drone at p (heading self.heading): the mat when wholly seen."""
        if not self.pad:
            return {'candidates': []}
        along_p, left_p = self.along(p), (p[1] - self.origin[1]) * self.f[0] - (p[0] - self.origin[0]) * self.f[1]
        rel = self.heading - self.yaw
        cam_a = along_p + CAM_FORWARD * math.cos(rel)
        cam_l = left_p + CAM_FORWARD * math.sin(rel)
        cam_u = p[2] + CAM_GROUND
        pa, pl, pyaw = self.pad
        c, s = math.cos(math.radians(pyaw)), math.sin(math.radians(pyaw))
        to_cam = np.linalg.inv(mount_matrix(PITCH))
        for da, db in ((0.425, 0.425), (0.425, -0.425), (-0.425, 0.425), (-0.425, -0.425)):
            ca, cl = pa + c * da - s * db, pl + s * da + c * db            # corner, start frame
            # into the camera's level frame (F along the heading, L left)
            va, vl = ca - cam_a, cl - cam_l
            f = va * math.cos(rel) + vl * math.sin(rel)
            l = -va * math.sin(rel) + vl * math.cos(rel)
            x, y, z = to_cam @ np.array([f, l, -cam_u])
            if z <= 0 or not (0 <= 489.6 * x / z + 320 < 640 and 0 <= 489.3 * y / z + 200 < 400):
                return {'candidates': []}
            if self.wall:                                                  # hidden behind the wall?
                near, thick, height = self.wall
                for wa in (near, near + thick):
                    if min(cam_a, ca) < wa < max(cam_a, ca):
                        if cam_u + (wa - cam_a) / (ca - cam_a) * (0.0 - cam_u) < height:
                            return {'candidates': []}
        va, vl = pa - cam_a, pl - cam_l
        f = va * math.cos(rel) + vl * math.sin(rel) + self.np_rng.normal(0, 0.02)
        l = -va * math.sin(rel) + vl * math.cos(rel) + self.np_rng.normal(0, 0.02)
        side = 0.85 / cam_u
        return {'candidates': [{'centre': [f / cam_u, l / cam_u], 'sides': [side, side], 'touches': False}]}

    # -- interface ---------------------------------------------------------------
    def take(self, command):
        if command is None:
            return
        kind, xyz = command
        self.published.append((round(self.t, 2), kind))
        self.last_sp = self.t
        if kind == 'pose':
            self.hold_pos, self.mode = self.envelope.clamp_position(xyz, self.origin).value, 'position'
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

    # -- PX4 + vehicle --------------------------------------------------------------
    def px4_step(self):
        t = self.t
        if not self.armed and t >= self.arm_at and self.armed_at is None:
            self.armed, self.nav, self.armed_at = True, ALTCTL, t
        if self.armed and t >= self.switch_at and self.nav == ALTCTL and self.offboard_at is None:
            self.nav = POSCTL
        if self.takeover_at is not None and t >= self.takeover_at and self.nav == OFFBOARD:
            self.nav = ALTCTL
        want = [0.0, 0.0, 0.0]
        if self.armed and self.nav == OFFBOARD:
            if self.mode == 'velocity':
                want = list(self.target_vel)
            elif self.hold_pos is not None:
                seen = self.reported(self.p)
                gain = (0.95, 0.95, 1.0)
                want = [gain[k] * (self.hold_pos[k] - seen[k]) for k in range(3)]
                h = math.hypot(want[0], want[1])
                if h > 12.0:
                    want[0], want[1] = want[0] * 12.0 / h, want[1] * 12.0 / h
                want[2] = max(-1.5, min(3.0, want[2]))
            if self.took_off_at is None and t - self.offboard_at < 1.5:
                want = [0.0, 0.0, 0.0]
            if self.mode != 'velocity' and self.tilt:
                # PX4 holds the *reported* position: a climb shifts the real one along the heading.
                want[0] += self.tilt * self.v[2] * self.f[0]
                want[1] += self.tilt * self.v[2] * self.f[1]
        if self.armed and self.nav == OFFBOARD and self.mission.hold_yaw is not None and self.took_off_at is not None:
            err = math.atan2(math.sin(self.mission.hold_yaw - self.heading), math.cos(self.mission.hold_yaw - self.heading))
            self.heading += max(-math.radians(60) * DT, min(math.radians(60) * DT, err))
        for k in range(3):
            self.v[k] += (want[k] - self.v[k]) * min(1.0, DT / 0.25)
        if self.p[2] <= 1e-6 and self.v[2] < 0:
            self.v = [0.0, 0.0, 0.0] if self.landed or self.took_off_at is None else [self.v[0] * 0.5, self.v[1] * 0.5, 0.0]
        for k in range(3):
            self.p[k] += self.v[k] * DT
        if self.p[2] < 0:
            self.p[2], self.v[2] = 0.0, 0.0
        if self.p[2] > 0.05 and self.took_off_at is None:
            self.took_off_at, self.landed = t, False
        descending = self.nav == OFFBOARD and self.mode == 'velocity' and self.target_vel[2] <= -0.275
        if self.armed and self.took_off_at is not None and not self.landed:
            if self.p[2] <= 1e-6 and descending:
                self.contact_since = t if self.contact_since is None else self.contact_since
                if t - self.contact_since >= 2.0:
                    self.landed, self.landed_at = True, t
                    self.events.append((round(t, 2), 'landing detected'))
            else:
                self.contact_since = None
        if self.armed and self.landed and self.landed_at is not None and t - self.landed_at >= 2.0:
            self.armed = False
            self.events.append((round(t, 2), 'auto-disarm: landed'))
        if self.armed and self.took_off_at is None and t - self.armed_at >= 10.0:
            self.armed = False
            self.events.append((round(t, 2), 'auto-disarm: no takeoff within 10 s'))
        if self.wall:                                     # body: 0.5 m long, landing gear to props
            a, z = self.along(), self.p[2]
            near, thick, height = self.wall
            if a + BODY_HALF_LENGTH > near and a - BODY_HALF_LENGTH < near + thick and z < height \
                    and z + 0.25 > self.wall_bottom:
                self.contacts += 1
            if self.crossbar:
                lo, hi, out = self.crossbar
                if a + BODY_HALF_LENGTH > near - out and a - BODY_HALF_LENGTH < near and z < hi and z + 0.25 > lo:
                    self.contacts += 1

    def run(self, duration=100.0):
        for i in range(int(round(duration / DT))):
            self.t = i * DT
            self.history.append(list(self.p))
            if i % 10 == 0:
                self.state = {'px4_link': True, 'armed': self.armed, 'landed': self.landed,
                              'offboard_active': self.nav == OFFBOARD, 'origin_enu': list(self.origin),
                              'local_enu': self.reported(self.p), 'received': self.t}
            if i % 2 == 0:
                self.odom = {'position': tuple(self.reported(self.p)), 'yaw': self.heading + 0.05,
                             'px4_yaw': self.heading, 'received': self.t}
            dropped = self.depth_dropout and self.depth_dropout[0] <= self.t < self.depth_dropout[1]
            if self.depth and not dropped and i % 5 == 0:      # 10 Hz, 0.1 s old
                seen_from = self.history[0]
                floor = {'height_m': round(seen_from[2] + CAM_GROUND + self.rng.gauss(0, 0.005), 3),
                         'pitch_deg': round(PITCH + self.rng.gauss(0, 0.3), 2)}
                if self.floor_fit:                      # a fit locked onto the wrong surface
                    floor = {'pitch_deg': self.floor_fit[0], 'height_m': self.floor_fit[1]}
                self.front = {'cells': self.cells(seen_from), 'top_edge_deg': TOP_EDGE, 'floor': floor,
                              'pitch_deg': PITCH, 'received': self.t}
            if self.pad_feed and i % 10 == 5:                 # 5 Hz, 0.1 s old
                self.pad_msg = dict(self.pad_view(self.history[-5] if len(self.history) >= 5 else self.p),
                                    received=self.t)
            if self.remote_fn and i % 5 == 2:                 # 10 Hz from the laptop
                msg = self.remote_fn(self.t, self.mission.phase, self)
                if msg is not None:
                    self.remote_msg = dict(msg, received=msg.get('received', self.t))
            if i % 5 == 0:
                phase = self.mission.phase
                self.take(self.mission.step(self.t, self.state, self.odom, self.front, self.pad_msg,
                                            self.remote_msg))
                if self.mission.phase in self.mission.FLYING:
                    self.held.append(self.mission.hold_yaw)
                for line in self.mission.log:
                    self.events.append((round(self.t, 2), '[%s] %s' % (self.mission.phase, line)))
                self.mission.log.clear()
                if self.wall and phase == 'HOLD_WALL' and self.stop_gap is None:
                    self.stop_gap = self.wall[0] - (self.along() + CAM_FORWARD)
                if self.wall and phase == 'CLIMB_OVER':
                    self.climb_alongs.append(self.along())
                if self.wall and phase == 'CLIMB_OVER' and self.p[2] < self.wall[2]:
                    gap = self.wall[0] - (self.along() + CAM_FORWARD)
                    self.max_gap_error = max(self.max_gap_error, abs(gap - 0.5))
                    self.min_gap = gap if self.min_gap is None else min(self.min_gap, gap)
                self.interface_tick()
            self.px4_step()
            self.trace.append((self.t, tuple(self.p), self.mission.phase))
        return self

    def log(self):
        return '\n'.join('%6.2f %s' % e for e in self.events)


class OverWallTest(unittest.TestCase):
    def assertLandedBeyond(self, w, clearance=0.2):
        self.assertFalse(w.armed, w.log())
        self.assertTrue(w.landed, w.log())
        self.assertEqual(w.contacts, 0, w.log())
        self.assertGreater(w.along() - BODY_HALF_LENGTH, w.wall[0] + w.wall[1] + clearance, w.log())
        self.assertIn('RESULT', w.log())

    def test_measures_the_wall_top_and_goes_over(self):
        w = World().run()
        self.assertLandedBeyond(w)
        self.assertGreater(w.stop_gap, 0.40, w.log())                 # camera to wall when stopped
        self.assertLess(w.stop_gap, 0.72, w.log())
        self.assertIn('Wall top in view', w.log())
        self.assertGreaterEqual(w.mission.result['wall_top_m'], 0.98, w.log())   # 5 cm cells read high,
        self.assertLessEqual(w.mission.result['wall_top_m'], 1.08, w.log())      # never low
        self.assertAlmostEqual(w.mission.result['camera_height_m'], CAM_GROUND, delta=0.02, msg=w.log())
        top = max(p[2] for _, p, _ in w.trace)
        self.assertGreater(top, w.wall[2] + 0.2, w.log())             # landing gear clears it
        self.assertEqual(w.mission.phase, 'WAIT', w.log())             # ready again

    def test_known_wall_height(self):
        w = World(params={'wall_height_m': 1.0}).run()
        self.assertLandedBeyond(w)
        self.assertNotIn('Wall top in view', w.log())

    def test_tilted_vio_frame_both_ways(self):
        # The stack's TF says the camera is level; it is pitched 27 deg. Holding the wall gap by
        # depth while climbing and landing 2 m past the stop point must cope either way.
        for tilt in (27.0, -27.0):
            w = World(vio_tilt_deg=tilt).run(110.0)
            self.assertLandedBeyond(w, clearance=0.1)
            # Climbing at the wall: the depth loop holds ~0.5 m; it must never close in.
            self.assertGreater(w.min_gap, 0.38, (tilt, w.log()))
            self.assertLess(w.max_gap_error, 0.25, (tilt, w.log()))

    def test_blank_wall_is_never_flown_into(self):
        # No texture, no stereo match: the wall itself is invisible. It still hides the floor
        # behind it, which stops the approach; the top cannot be measured, so it climbs to the
        # height limit and goes over.
        w = World(wall_textured=False).run(110.0)
        self.assertLandedBeyond(w)
        self.assertIn('floor', w.mission.result.get('stopped_on', ''), w.log())

    def test_wall_too_tall_lands_in_front(self):
        w = World(wall_height=2.6).run()
        self.assertEqual(w.contacts, 0, w.log())
        self.assertFalse(w.armed, w.log())
        self.assertLess(w.along() + CAM_FORWARD, w.wall[0], w.log())
        self.assertIn('too tall', w.log())

    def test_no_wall_lands_at_max_forward(self):
        w = World(wall_at=None).run()
        self.assertFalse(w.armed, w.log())
        self.assertAlmostEqual(w.along(), w.mission.p.max_forward_m, delta=0.35, msg=w.log())   # 4.57 m = 15 ft
        self.assertIn('No wall within', w.log())

    def test_false_matches_alone_never_stop_it(self):
        w = World(wall_at=None, spurious=400).run()
        self.assertNotIn('[HOLD_WALL]', w.log())
        self.assertIn('No wall within', w.log())

    def test_no_depth_node_does_not_approach(self):
        w = World(depth=False).run(40.0)
        self.assertFalse(w.armed, w.log())
        self.assertLess(abs(w.along()), 0.2, w.log())
        self.assertIn('No depth', w.log())

    def test_depth_lost_mid_approach_stops_short(self):
        w = World(depth_dropout=(14.0, 1e9)).run(60.0)
        self.assertEqual(w.contacts, 0, w.log())
        self.assertFalse(w.armed, w.log())
        self.assertLess(w.along() + CAM_FORWARD, w.wall[0], w.log())
        self.assertIn('Depth lost', w.log())

    def test_pilot_takeover_stops_publishing(self):
        w = World(takeover_at=15.0).run(30.0)
        self.assertEqual(w.mission.phase, 'ABORTED')
        self.assertEqual([t for t, _ in w.published if t > 15.3], [])

    def test_height_offset_between_px4_and_vision(self):
        for offset in (-0.4, 0.2):
            w = World(local_offset_z=offset).run(110.0)
            self.assertLandedBeyond(w)

    def test_wall_close_to_the_start(self):
        w = World(wall_at=1.2).run()
        self.assertLandedBeyond(w)

    def test_course_wall_1_6_m(self):
        # The test net: a textured wall ~1.6 m tall; backstop 15 ft. The top is measured by camera.
        for wall_at in (2.5, 4.0):
            w = World(wall_at=wall_at, wall_height=1.6).run(120.0)
            self.assertLandedBeyond(w)
            self.assertIn('Wall top in view', w.log())
            self.assertGreaterEqual(w.mission.result['wall_top_m'], 1.58, w.log())
            self.assertLessEqual(w.mission.result['wall_top_m'], 1.68, w.log())
            self.assertGreater(max(p[2] for _, p, _ in w.trace), 1.6 + 0.2, w.log())

    def test_never_closer_than_the_clearance_to_the_net(self):
        net = 2.896
        for kw in ({}, {'background_at': None}, {'wall_height': 1.6, 'wall_at': 3.5}):
            w = World(**kw).run(120.0)
            drone_top = max(p[2] for _, p, _ in w.trace) + (CAM_GROUND - 0.03) + 0.08
            self.assertLessEqual(drone_top, net - 0.4 + 0.05, (kw, w.log()))

    def test_low_net_turns_a_tall_wall_back(self):
        # A net only 2.2 m up leaves no room over a 1.6 m wall: land in front, never climb into it.
        w = World(wall_height=1.6, params={'net_top_m': 2.2}).run(120.0)
        self.assertEqual(w.contacts, 0, w.log())
        self.assertFalse(w.armed, w.log())
        self.assertLess(w.along() + CAM_FORWARD, w.wall[0], w.log())
        self.assertLessEqual(max(p[2] for _, p, _ in w.trace) + CAM_GROUND - 0.03 + 0.08, 2.2 - 0.4 + 0.05, w.log())

    def test_a_wrong_floor_fit_is_ignored(self):
        # Seen on skybeta-03: a fit at 51.8 deg / 0.19 m when little flat floor was in view.
        w = World(floor_fit=(51.8, 0.19), params={'camera_height_m': CAM_GROUND}).run()
        self.assertEqual(w.mission.result['camera_height_m'], CAM_GROUND, w.log())
        self.assertIn('camera_height_m (no floor fit)', w.log())
        self.assertLandedBeyond(w)

    def test_holds_px4_start_heading(self):
        # Odometry's yaw is 0.05 rad off PX4's here: the held heading and the path follow PX4's.
        w = World().run()
        self.assertLandedBeyond(w)
        self.assertIn('(held)', w.log())
        self.assertTrue(w.held, w.log())
        self.assertTrue(all(y == w.yaw for y in w.held), set(w.held))
        self.assertAlmostEqual(math.atan2(w.mission.forward[1], w.mission.forward[0]), w.yaw)
        self.assertIsNone(w.mission.hold_yaw)                 # none once landed (DONE / WAIT)

    COURSE = {'start_to_wall_m': 0.686, 'max_forward_m': 0.25}     # config/overwall.yaml

    def test_course_start_27_in_from_the_wall_lands_6_ft_2_in_past_it(self):
        w = World(wall_at=0.686, wall_height=1.6, wall_bottom=0.3, params=self.COURSE).run(120.0)
        self.assertLandedBeyond(w)
        self.assertAlmostEqual(w.along(), 0.686 + 1.880, delta=0.2, msg=w.log())
        self.assertIn('Wall top in view', w.log())
        self.assertTrue(w.held and all(y == w.yaw for y in w.held), w.log())     # heading held all flight
        self.assertLess(w.mission.result['wall_at_forward_m'], 0.25, w.log())    # no creep: it was there

    def test_course_distance_measured_by_depth_not_assumed(self):
        # The 27 in may be taken to the drone's front, not its centre: depth decides.
        w = World(wall_at=0.9, wall_height=1.6, wall_bottom=0.3, params=self.COURSE).run(120.0)
        self.assertLandedBeyond(w)
        self.assertAlmostEqual(w.along(), 0.9 + 1.880, delta=0.2, msg=w.log())
        self.assertAlmostEqual(w.mission.result['wall_from_start_m'], 0.9, delta=0.1, msg=w.log())   # depth's, not 0.686

    def test_course_cross_bar_does_not_push_it_back(self):
        # A 20 cm bar standing 10 cm out of the wall: the camera passes it on the climb. Start right
        # at the 0.5 m camera gap, where an unfiltered hold backs off as the bar goes by.
        bar = dict(wall_at=0.62, wall_height=1.6, wall_bottom=0.3, crossbar=(0.80, 1.00, 0.10))
        raw = World(**bar, params=dict(self.COURSE, keep_window=1, keep_deadband_m=0.0, keep_max_mps=0.15)).run(120.0)
        self.assertGreater(raw.climb_alongs[0] - min(raw.climb_alongs), 0.03, raw.log())   # the jump, unfiltered
        w = World(**bar, params=self.COURSE).run(120.0)
        self.assertLandedBeyond(w)
        self.assertAlmostEqual(w.along(), 0.62 + 1.880, delta=0.25, msg=w.log())
        self.assertLess(w.climb_alongs[0] - min(w.climb_alongs), 0.02, w.log())          # filtered: no jump

    def test_course_blank_wall_is_never_touched(self):
        # A blank wall: whichever safe way it goes (land in front, or stop on the floor seen through
        # the hole and go over at the height limit), it must never touch the wall.
        w = World(wall_at=0.686, wall_height=1.6, wall_bottom=0.3, wall_textured=False,
                  params=self.COURSE).run(120.0)
        self.assertEqual(w.contacts, 0, w.log())
        self.assertFalse(w.armed, w.log())
        self.assertTrue(w.along() < 0.686 - BODY_HALF_LENGTH or w.along() - BODY_HALF_LENGTH > 0.686 + 0.1, w.log())

    def landed_miss(self, w):
        """Distance from the drone's centre to the mat's centre, on the floor."""
        left = (w.p[1] - w.origin[1]) * w.f[0] - (w.p[0] - w.origin[0]) * w.f[1]
        return math.hypot(w.along() - w.pad[0], left - w.pad[1])

    COURSE_WALL = dict(wall_at=0.686, wall_height=1.6, wall_bottom=0.3)

    def test_course_lands_on_the_pad_it_sees(self):
        # The pad is not where land_beyond_wall_m says (6 ft 2 in past the wall, on the centreline):
        # 0.35 m further, 0.35 m to the left, turned 25 deg. Rule 5: the whole drone inside the
        # pad's ring, so the centre must come down on the mat's centre, not at the set distance.
        w = World(**self.COURSE_WALL, pad=(0.686 + 1.88 + 0.35, 0.35, 25.0), params=self.COURSE).run(120.0)
        self.assertLandedBeyond(w)
        self.assertLess(self.landed_miss(w), 0.15, w.log())
        self.assertIn('Pad seen', w.log())

    def test_pad_off_to_the_side_is_found_by_the_sweep(self):
        # The mat 1.1 m to the left: out of the colour camera's +-33 deg from the look point.
        # One +-35 deg sweep brings it in (the user's choice: search, then land).
        w = World(**self.COURSE_WALL, pad=(0.686 + 1.7, 1.1, 0.0), params=self.COURSE).run(140.0)
        self.assertLandedBeyond(w)
        self.assertIn('Sweeping', w.log())
        self.assertLess(self.landed_miss(w), 0.15, w.log())

    def test_pad_never_seen_lands_at_the_set_distance(self):
        # pad_detect runs but the mat is nowhere: sweep once, then land 6 ft 2 in past the wall
        # as before, the heading back on the start heading.
        w = World(**self.COURSE_WALL, pad_feed=True, params=self.COURSE).run(140.0)
        self.assertLandedBeyond(w)
        self.assertIn('Sweeping', w.log())
        self.assertIn('not seen', w.log())
        self.assertAlmostEqual(w.along(), 0.686 + 1.880, delta=0.2, msg=w.log())
        self.assertAlmostEqual(w.heading, w.yaw, delta=math.radians(3), msg=w.log())

    # -- the PS5 controller (rules now allow remote input): nudge + buttons, mission keeps flying --
    def left_of_start(self, w):
        return (w.p[1] - w.origin[1]) * w.f[0] - (w.p[0] - w.origin[0]) * w.f[1]

    def test_remote_nudge_puts_it_on_the_real_pad(self):
        # Seen 2026-11-07: a blue ladder beside the pad was taken for it. Here the detector's pad
        # IS the ladder (0.6 m right of the real mat); the operator holds L1 + left stick for
        # 2.4 s over it (0.25 m/s) and the drone lands on the real mat instead.
        ladder, real = (0.686 + 1.88, -0.3), (0.686 + 1.88, 0.3)
        held = {}

        def operator(t, phase, w):
            if phase == 'ARRIVE' and 'from' not in held:
                held['from'] = t
            on = 'from' in held and t - held['from'] < 2.4
            return {'sticks': [0.0, 1.0 if on else 0.0, 0.0], 'events': []}
        w = World(**self.COURSE_WALL, pad=(ladder[0], ladder[1], 0.0), remote=operator, params=self.COURSE).run(140.0)
        self.assertLandedBeyond(w)
        self.assertAlmostEqual(self.left_of_start(w), real[1], delta=0.15, msg=w.log())
        self.assertAlmostEqual(w.along(), real[0], delta=0.15, msg=w.log())

    def test_remote_land_here_at_the_wall(self):
        # L1 + cross while it creeps at the wall: it lands where it is, in front of the wall.
        def operator(t, phase, w):
            return {'sticks': [0.0, 0.0, 0.0], 'events': [[1, 'land']] if phase == 'HOLD_WALL' else []}
        w = World(wall_at=1.2, wall_height=1.6, wall_bottom=0.3, remote=operator).run(80.0)
        self.assertEqual(w.contacts, 0, w.log())
        self.assertFalse(w.armed, w.log())
        self.assertLess(w.along() + CAM_FORWARD, 1.2, w.log())
        self.assertIn('Land here', w.log())
        self.assertNotIn('[CLIMB_OVER]', w.log())

    def test_remote_hold_then_resume(self):
        # Circle in OVER pauses it in place for 5 s; triangle resumes; it finishes as without.
        mark = {}

        def operator(t, phase, w):
            if phase == 'OVER' and 'hold' not in mark:
                mark['hold'], mark['along'] = t, w.along()
            ev = []
            if 'hold' in mark:
                ev.append([1, 'hold'])
                if t - mark['hold'] > 5.0:
                    ev.append([2, 'resume'])
                elif t - mark['hold'] > 1.0:
                    mark['drift'] = max(mark.get('drift', 0.0), abs(w.along() - mark['along2']) if 'along2' in mark else 0.0)
                    mark.setdefault('along2', w.along())
            return {'sticks': [0.0, 0.0, 0.0], 'events': ev}
        w = World(**self.COURSE_WALL, remote=operator, params=self.COURSE).run(140.0)
        self.assertLandedBeyond(w)
        self.assertLess(mark.get('drift', 9.0), 0.05, w.log())                       # held still
        self.assertAlmostEqual(w.along(), 0.686 + 1.880, delta=0.2, msg=w.log())     # then as usual

    def test_stale_remote_is_ignored(self):
        # The laptop's last message (stick hard left) is 1 s old: the link died. No nudge.
        def operator(t, phase, w):
            return {'sticks': [0.0, 1.0, 0.0], 'events': [], 'received': max(0.0, t - 1.0)}
        w = World(**self.COURSE_WALL, remote=operator, params=self.COURSE).run(120.0)
        self.assertLandedBeyond(w)
        self.assertAlmostEqual(self.left_of_start(w), 0.0, delta=0.1, msg=w.log())

    def test_remote_nudge_is_capped(self):
        # Stick held left the whole flight: the offset stops at nudge_max_xy_m (1.5 m).
        def operator(t, phase, w):
            return {'sticks': [0.0, 1.0, 0.0], 'events': []}
        w = World(**self.COURSE_WALL, remote=operator, params=self.COURSE).run(120.0)
        self.assertFalse(w.armed, w.log())
        self.assertLess(self.left_of_start(w), 1.5 + 0.15, w.log())
        self.assertGreater(self.left_of_start(w), 1.2, w.log())

    def test_param_guards(self):
        with self.assertRaises(ValueError):
            Params(descent_slow_mps=0.2)
        with self.assertRaises(ValueError):
            Params(cruise_height_m=2.5, max_height_m=2.2)
        with self.assertRaises(ValueError):
            Params(net_top_m=1.2)


if __name__ == '__main__':
    unittest.main()
