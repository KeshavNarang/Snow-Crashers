"""An irregular, tarp-covered version of the published course, for testing.

The real course is not built to the CAD: freestanding walls are feet off,
pivot where they meet the barrier or the net, kink and lean; everything is
draped in tarps that bulge, pool at the floor and hang past the wall ends;
the tunnel is about 1 ft short with a tarp hanging off its mouth; the net is
nearly invisible with a cluttered room behind it. ``generate(seed)`` builds
one such course from the CAD's topology:

- an OBJ (walls, tarps, tunnel, bevel, boulders, floor; no nets) for the
  voxel renderer and the contact judge;
- metadata: true launch placement, net planes, pad, moving clutter, and the
  checkpoint lines moved with the objects they belong to.

Layouts that close a passage, open a way around the tunnel, or push a wall
out of the net are rejected and resampled.
"""
from dataclasses import dataclass, field, replace
import json
import math
from pathlib import Path

import numpy as np

from drone_ref.course import read_obj
from localsim import messy_frame as mf
from localsim import messy_ladder as ml
from localsim import messy_mesh as mm
from localsim import messy_stand as ms

ROOT = Path(__file__).resolve().parents[1]
CAD = ROOT / 'assets/course-v3.obj'
NET = 3.048
WALL_T, WALL_H = 0.1016, 1.727
BARRIER_Y = 0.4572                # CAD: the barrier's centreline
SCAN_BARRIER_Y = 0.41             # as scanned
TUBE = {'cx': 2.3114, 'cz': 0.9398, 'hw': 0.7366, 'hh': 0.9144, 'y_south': -0.3048, 'y_north': 1.2192}
LAUNCH = (-2.5146, 0.9144)
PAD = (-2.04216, -0.79248, 0.76188)
BOULDERS = [(2.01, -2.47), (2.53, -1.55), (0.79, -2.35)]    # in the east room; more by the mat (PAD_ROCKS)
# Walls: CAD group, the end that is attached ('barrier', 'north'/'south' net, or None).
# As scanned (2026-11-06); LAYOUT below rebuilds the maze from these.
WALLS = {'M1': ('M1 maze divider', 'barrier'), 'M2': ('M2 maze divider', 'north'),
         'M3': ('M3 maze divider', 'barrier'), 'C1': ('C1 chicane wall', 'barrier'),
         'A1': ('A1 approach wall', 'barrier'), 'C2a': ('C2a chicane wall (window)', 'south')}
# As scanned (2026-11-06, assets/course-scan.npz from tools/course_scan.py): the window wall is
# one long panel where the CAD's C2a is (no C2b), its south end inside the net.
# Floor level under the drone on the launch mark (its arms reach 0.20 m; localsim.goalrun
# drone_contacts): a stand batten there is under the drone, not beside it.
LAUNCH_FLOOR_CLEAR = 0.22
BODY_RADIUS = 0.29                 # Pixhawk to the rear rotor tips (Sky Beta CAD, 7 in props)
LONG_PANEL, SHORT_PANEL = 1.613, 1.029   # A1, by the pad, is short
WINDOW_WALL = 1.78                # scanned length, the vinyl wrapping its ends
WINDOW_WALL_INSET = (0.03, 0.10)  # its south end this far inside the south net
# Walls the scan places away from the CAD (m, CAD axes): the maze's two
# eastern dividers stand further east. Everything can still move: offsets
# from here are the severity-scaled jitter.
SCAN_SHIFT = {'M2': (0.29, 0.), 'M3': (0.70, 0.)}
# ...and turned from the CAD (degrees, counter-clockwise): M1 leans its free
# (north) end east.
SCAN_YAW = {'M1': -6.}
# Wall layouts. 'scan': the walls as scanned (above). 'rebuilt': the maze as
# rebuilt on 2026-11-07 (photos/Changed_Walls.jpeg, photos/New_Walls_*;
# measured in agents/logs/2026-11-07-walls-rebuilt.md). LAYOUT is the default;
# generate(..., layout=...) and goalrun --layout pick one per run. Either way
# the floor tape and the lines tied to it (CP1, gate A) follow the walls as
# scanned: the tape was not re-laid with the rebuild.
LAYOUTS = ('scan', 'rebuilt')
LAYOUT = 'rebuilt'
# The rebuild (each range: its middle at severity 0, all of it at severity 1):
# - M2, the north-net divider the drone went around second, is gone.
# - M1, the first wall it goes around, still stands on its barrier joint but is
#   swung 30-60 degrees from north-south, its free (north) end east.
M1_TILT_DEG = (30., 60.)
# - M3, the third, is now a short panel standing free ~0.3 m north of the
#   barrier, swung the same way: its stand's two pavers, on its north-west side,
#   are at (0.18, 0.97) and (0.64, 1.62) in both overhead photos (2 cm apart),
#   36.3 and 34.7 degrees east of north. 0.79 m apart, they fit a short panel's
#   end stands (a long one's would be ~1.5 m apart); the panel is placed so its
#   pavers (PAVER_OUT off its centreline, 6 cm in from its ends, messy_stand)
#   land on them. Centre and turn below; the usual placement jitter on top.
M3_CENTRE = (0.65, 1.13)
M3_FROM_NORTH_DEG = 35.5
PAVER_OUT = 0.29                  # a paver's centre off the panel's centreline (stand feet, messy_stand)
# - A long panel stands across the north end of the first corridor, ~0.4 m in
#   from the north net, its stand's feet toward the corridor (south). Centre and
#   turn (degrees CCW from +x) from its post feet in both overhead photos, 1.21-1.25 m
#   apart: too far apart for a short panel, a long one with its stands set ~0.15 m in.
NW_PANEL_CENTRE = (-2.00, 2.67)
NW_PANEL_YAW_DEG = -3.5
# The barrier's panels as scanned, from the west net (degrees, counter-clockwise):
# the first, behind the launch, rises toward M1; the second comes back down.
BARRIER_TILT = (6., -2., 0.)
BARRIER_WEST_Y = SCAN_BARRIER_Y - 0.10
# Where each stands along the barrier, in joints (1.5: the middle of the second
# panel). M1 at the end of the first short panel; A1, by the pad, about half a
# short panel west of the second joint (team observation).
BARRIER_JOINT = {'M1': 1, 'A1': 1.5}
CP1_FROM_NET = 3.048 - 2.584
# C1, on the right leaving the tunnel, leans the other way from the CAD (less
# steeply): toward the tunnel exit, narrowing it, degrees from straight south.
C1_LEAN_DEG = (8., 15.)
# The window (scan): centred on its wall, 0.90 wide, sill 0.87 m, 0.62 m tall.
WINDOW_SHIFT = (-0.04, 0.04)     # along the wall from its middle
WINDOW_WIDTH = (0.80, 0.95)      # the width varies (team); 0.58 is the rotor span
WINDOW_HEIGHT = (0.58, 0.66)
WINDOW_SILL = (0.80, 0.92)
# Placement offsets (positions and turns) are this much wider than first
# estimated: the course has been seen rebuilt differently several times.
JITTER = 1.3
# The landing mat (team): somewhere in the landing area's south-east (+x -y)
# corner, and it has moved between visits, so anywhere in this box (centre,
# not scaled by severity) clear of the window wall and inside the net.
PAD_AREA = ((-2.35, -0.85), (-NET + 0.45, -1.55))
PAD_FROM_WINDOW_WALL = 0.80             # mat centre to the wall's centreline (feet, struts, tarp)
# Three large rocks around the mat (team), anywhere around it: each one's near
# edge at least ROCK_CLEAR from the mat's edge (never on the mat; user's hard
# limit, 1.5 ft), and no further than PAD_ROCK_REACH beyond that ("nearby").
PAD_ROCKS = 3
ROCK_CLEAR = 0.4572                     # 1.5 ft from the mat's square edge to the rock's edge, hard minimum
PAD_ROCK_REACH = 0.6
# Around the net (team): the next net 3 ft north (net to net), empty but for a
# flat blue floor tarp and its PVC frame; the hall's glass wall 8 ft west of the west net.
NEXT_COURSE_GAP = 0.914
NORTH_NET_SLACK = 0.10            # the north net may hang this far in (validate: gates against it)
GLASS_FROM_NET = 2.438
# The tunnel sits slightly lower than the model: its top about 10 cm down.
TUNNEL_DROP = (0.07, 0.13)
# The black tape line on the floor through the maze (CAD 'Guideline (zone 1)', 2 in wide).
GUIDELINE = [(-2.515, 0.889), (-2.515, 2.585), (-1.372, 2.585), (-1.372, 0.972), (-0.254, 0.972),
             (-0.254, 2.585), (0.330, 2.585)]
GUIDELINE_WIDTH = 0.051


def _rot(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def _cad_wall(mesh, group):
    """(attached-end-agnostic) centreline ends a, b of a CAD slab."""
    points = np.unique(np.round(np.asarray(mesh.vertices)[np.unique(mesh.groups[group])][:, :2], 4), axis=0)
    centre = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centre)
    axis = vt[0]
    u = (points - centre) @ axis
    return centre + axis * u.min(), centre + axis * u.max()


@dataclass
class Wall:
    name: str
    pieces: list                  # [(a, b)] centrelines after kinks
    lean: float
    height: float
    hole: tuple = None            # ((u0, u1), (z0, z1)) on the single piece
    flaps: tuple = (0., 0.)
    free_end: np.ndarray = None
    yaw: float = 0.
    gap: float = 0.25             # panel bottom above the floor
    tarp_side: int = 1            # which face carries the printed tarp
    print: str = 'scifi'
    cad_pivot: np.ndarray = None  # where the CAD put the end it pivots about
    pivot: np.ndarray = None


@dataclass
class Course:
    seed: int
    severity: float
    groups: dict
    walls: list
    tunnel: dict
    pad: tuple
    launch: dict
    nets: dict
    clutter: list
    lines: dict = field(default_factory=dict)
    boulders: list = field(default_factory=list)


class Sampler:
    def __init__(self, seed, severity):
        self.rng = np.random.default_rng(seed)
        self.s = severity
        self.cad = read_obj(CAD)
        # The walls are one build: the same panels, stands and base gap.
        # Two panel lengths, each the same everywhere (the CAD's 63.5 in and 40.5 in).
        self.panel = {'long': LONG_PANEL * self.u(0.97, 1.03, False), 'short': SHORT_PANEL * self.u(0.95, 1.05, False),
                      'height': WALL_H + self.u(-0.03, 0.02, False), 'gap': self.u(0.2, 0.3, False),
                      'strut_top': None, 'strut_out': self.u(0.26, 0.32, False)}

    def u(self, low, high, scale=True):
        """Uniform in [low, high]; ``scale``: a placement offset, by severity and JITTER."""
        k = self.s * JITTER if scale else 1.
        return float(self.rng.uniform(low * k, high * k))

    # -- the barrier and the tunnel --------------------------------------------
    def tunnel(self):
        L = TUBE['y_north'] - TUBE['y_south']
        cut = self.u(0.25, 0.35, scale=False)
        end = 'south' if self.rng.random() < 0.5 else 'north'
        yaw = math.radians(self.u(-5, 5))
        shift = np.array([self.u(-0.12, 0.04), self.u(-0.3, 0.3)])
        centre = np.array([TUBE['cx'], (TUBE['y_north'] + TUBE['y_south']) / 2])
        scale = (L - cut) / L
        fixed = TUBE['y_north'] if end == 'south' else TUBE['y_south']

        top = 1.854
        zscale = (top - self.u(*TUNNEL_DROP, scale=False)) / top

        def local(points):
            p = np.array(points, float)
            p[..., 1] = fixed + (p[..., 1] - fixed) * scale
            if p.shape[-1] == 3:
                p[..., 2] *= zscale
            return p

        def place(points):
            p = local(points)
            xy = (p[..., :2] - centre) @ _rot(yaw).T + centre + shift
            return np.concatenate([xy, p[..., 2:]], axis=-1) if p.shape[-1] == 3 else xy
        groups = {}
        vertices = np.asarray(self.cad.vertices)
        for name, tris in self.cad.groups.items():
            if name.startswith('Tube'):
                index = np.unique(tris)
                remap = {i: k for k, i in enumerate(index)}
                groups[name] = (place(vertices[index]), np.vectorize(remap.get)(np.asarray(tris)))
        mouths = {}
        for which, y, out in (('north', TUBE['y_north'], 1), ('south', TUBE['y_south'], -1)):
            mouth_y = fixed + (y - fixed) * scale
            mouths[which] = {'y_local': mouth_y, 'outward': out}
        for which, m in mouths.items():
            if which == 'north' or self.rng.random() < 0.5:
                v, t = mm.bevel(TUBE['cx'], m['y_local'], m['outward'], TUBE['cz'] * zscale, TUBE['hw'],
                                TUBE['hh'] * zscale, drop=self.u(0.3, 0.6, False), reach=self.u(0.15, 0.4, False),
                                rng=self.rng)
                xy = (v[:, :2] - centre) @ _rot(yaw).T + centre + shift
                groups[f'Tunnel tarp {which}'] = (np.column_stack([xy, v[:, 2]]), t)
                m['bevel'] = True
        groups['Tube hoops'] = ms.hoops(place, TUBE, TUBE['y_south'], TUBE['y_north'])
        groups['Tunnel drape'] = ms.drape(place, TUBE, TUBE['y_south'] + self.u(0., 0.5, False),
                                          TUBE['y_north'], self.rng)
        groups['Boulder tunnel corners'], corner_rocks = ms.corner_rocks(place, TUBE, mouths, self.rng)
        info = {'yaw': yaw, 'shift': shift.tolist(), 'cut': cut, 'cut_end': end, 'scale': scale, 'zscale': zscale,
                'corner_rocks': corner_rocks,
                'fixed_y': fixed, 'centre': centre.tolist()}
        for which, m in mouths.items():
            mid = np.array([TUBE['cx'], m['y_local']])
            m['xy'] = ((mid - centre) @ _rot(yaw).T + centre + shift).tolist()
            m['axis'] = (_rot(yaw) @ np.array([0., m['outward']])).tolist()
        info['mouths'] = mouths
        # Inside the east net, tarps and corner rocks included: slide it west if not.
        excess = max(float(v[:, 0].max()) for v, _ in groups.values()) - (NET - 0.03)
        if excess > 0:
            for name, (v, t) in list(groups.items()):
                groups[name] = (v - np.array([excess, 0., 0.]), t)
            shift[0] -= excess
            for m in mouths.values():
                m['xy'][0] -= excess
            for rock in corner_rocks:
                rock['xy'] = [rock['xy'][0] - excess, rock['xy'][1]]
            info['shift'] = shift.tolist()
        info['place_xy'] = lambda p: (np.asarray(p, float) - centre) @ _rot(yaw).T + centre + shift
        return groups, info

    def barrier(self, tunnel):
        j0 = np.array([-NET + 0.02, BARRIER_WEST_Y + self.u(-0.1, 0.1)])      # from just inside the west net
        # As built, seen from the pad: two short panels from the west net (M1
        # stands on the maze side at the end of the first, A1 on the landing
        # side at the end of the second), then a long one, then one cut to the tunnel.
        # Each joint a few cm off the scanned line, not a random walk of panel
        # angles: the panels butt end to end and the last meets the tunnel.
        joints, nominal = [j0], j0
        for length, tilt in zip((self.panel['short'], self.panel['short'], self.panel['long']), BARRIER_TILT):
            nominal = nominal + length * np.array([math.cos(math.radians(tilt)), math.sin(math.radians(tilt))])
            dy = nominal[1] + self.u(-0.04, 0.04) - joints[-1][1]
            joints.append(joints[-1] + np.array([math.sqrt(length ** 2 - dy ** 2), dy]))
        joints.append(tunnel['place_xy'](np.array([TUBE['cx'] - TUBE['hw'] + 0.04, BARRIER_Y])))
        walls = []
        for k, (a, b) in enumerate(zip(joints, joints[1:])):
            d = (b - a) / np.linalg.norm(b - a)
            ends = (a, b) if k < 3 else (a - d * 0.05, b + d * 0.05)     # Only the cut panel bridges.
            walls.append(self._dress(Wall(f'Barrier {k + 1}', [ends],
                                          math.radians(self.u(-1, 1, False)), self.panel['height'])))
        return walls, joints

    def _dress(self, wall):
        """Stand details seen in the photos: base gap, one tarped face, print."""
        wall.gap = self.panel['gap'] + self.u(-0.01, 0.01, False)
        wall.tarp_side = int(self.rng.choice([-1, 1]))
        wall.print = 'scifi' if self.rng.random() < 0.6 else 'rock'
        return wall

    @staticmethod
    def barrier_y(joints, x):
        xs, ys = [j[0] for j in joints], [j[1] for j in joints]
        return float(np.interp(x, xs, ys))

    # -- walls -----------------------------------------------------------------
    def wall(self, key, joints, placed=None):
        group, host = WALLS[key]
        a, b = _cad_wall(self.cad, group)
        shift = np.asarray(SCAN_SHIFT.get(key, (0., 0.)))
        a, b = a + shift, b + shift
        if host == 'barrier':
            pivot, free = (a, b) if abs(a[1] - BARRIER_Y) < abs(b[1] - BARRIER_Y) else (b, a)
        elif host in ('north', 'south'):
            target = NET if host == 'north' else -NET
            pivot, free = (a, b) if abs(a[1] - target) < abs(b[1] - target) else (b, a)
        else:                          # Continues another wall from its free end.
            host_cad = _cad_wall(self.cad, WALLS[host][0])
            host_free = max(host_cad, key=lambda p: p[1])
            pivot, free = (a, b) if np.linalg.norm(a - host_free) < np.linalg.norm(b - host_free) else (b, a)
        nominal, cad_pivot = free - pivot, pivot.copy()
        length = self.panel['short' if np.linalg.norm(nominal) < 1.3 else 'long']
        if key == 'C2a':
            length = WINDOW_WALL * self.panel['long'] / LONG_PANEL
        if key == 'C1':
            # As built: the CAD's lean mirrored (less steep) about the panel's
            # middle, so its foot leans toward the tunnel exit and narrows it.
            lean = math.radians(self.u(*C1_LEAN_DEG, scale=False))
            middle = (pivot + free) / 2
            nominal = np.array([math.sin(lean), -math.cos(lean)]) * np.linalg.norm(nominal)
            pivot = middle - nominal / 2
        yaw = math.radians(SCAN_YAW.get(key, 0.) + self.u(-4, 4))
        direction = _rot(yaw) @ (nominal / np.linalg.norm(nominal))
        if host == 'barrier' and key in BARRIER_JOINT:
            k = BARRIER_JOINT[key]
            lo, hi = joints[int(k)], joints[min(int(k) + 1, len(joints) - 1)]
            x = (lo + (k - int(k)) * (hi - lo))[0] + self.u(-0.03, 0.03, False)
            side = math.copysign(1., free[1] - BARRIER_Y)
            pivot = np.array([x, self.barrier_y(joints, x) + side * (WALL_T / 2 - 0.03)])
        elif host == 'barrier':
            x = pivot[0] + self.u(-0.12, 0.12)
            side = math.copysign(1., free[1] - BARRIER_Y)
            pivot = np.array([x, self.barrier_y(joints, x) + side * (WALL_T / 2 - 0.03)])
        elif host in ('north', 'south'):
            pivot = pivot + np.array([self.u(-0.12, 0.12), 0.])
            inset = self.u(*WINDOW_WALL_INSET, scale=False) if key == 'C2a' else 0.02
            pivot[1] = math.copysign(NET - inset, pivot[1])
        else:
            pivot = placed[host].free_end.copy()
        hole = None
        if key == 'C2a':
            width = self.u(*WINDOW_WIDTH, scale=False)
            centre = length / 2 + self.u(*WINDOW_SHIFT, scale=False)
            bottom = self.u(*WINDOW_SILL, scale=False)
            hole = ((centre - width / 2, centre + width / 2), (bottom, bottom + self.u(*WINDOW_HEIGHT, scale=False)))
        pieces = [(pivot, pivot + direction * length)]
        flaps = (0., 0.)                              # The vinyl wraps the ends; it does not hang past them.
        free_end = pieces[-1][1]
        wall = Wall(key, pieces, math.radians(self.u(-1, 1, False)), self.panel['height'],
                    hole, flaps, free_end, yaw, cad_pivot=cad_pivot, pivot=pieces[0][0].copy())
        self._dress(wall)
        if hole is not None:
            wall.gap = min(wall.gap, hole[1][0] - 0.1)       # The window's sill is panel.
            # Its vinyl faces the tunnel (east).
            d = pieces[0][1] - pieces[0][0]
            wall.tarp_side = 1 if np.array([-d[1], d[0]])[0] > 0 else -1
        return wall

    # -- the rebuild (LAYOUT 'rebuilt') ----------------------------------------
    def _middle_out(self, low, high):
        """Within [low, high]: its middle at severity 0, all of it at severity 1."""
        half = (high - low) / 2
        return low + half + float(np.clip(self.u(-half, half), -half, half))

    @staticmethod
    def _swung(wall, pivot, from_north_deg, length):
        """``wall`` standing at ``pivot``, its free end ``from_north_deg`` east of north."""
        t = math.radians(from_north_deg)
        direction = np.array([math.sin(t), math.cos(t)])
        free_end = pivot + direction * length
        return replace(wall, pieces=[(pivot.copy(), free_end)], free_end=free_end,
                       yaw=math.atan2(direction[1], direction[0]) - math.pi / 2, pivot=pivot.copy())

    @staticmethod
    def _feet_toward(wall, side):
        """Its stand's struts, feet and pavers (the bare face, messy_stand) on ``side``."""
        a, b = (np.asarray(p) for p in wall.pieces[0])
        normal = np.array([-(b - a)[1], (b - a)[0]])
        wall.tarp_side = -1 if normal @ np.asarray(side) > 0 else 1
        return wall

    def rebuild(self, scanned, joints):
        """The maze as rebuilt on 2026-11-07, from the walls as scanned (by name)."""
        walls = {k: w for k, w in scanned.items() if k != 'M2'}
        m1 = scanned['M1']
        walls['M1'] = self._swung(m1, m1.pivot, self._middle_out(*M1_TILT_DEG),
                                  float(np.linalg.norm(m1.free_end - m1.pivot)))
        centre = np.asarray(M3_CENTRE) + np.array([self.u(-0.1, 0.1), self.u(-0.1, 0.1)])
        from_north = M3_FROM_NORTH_DEG + self.u(-4, 4)
        t = math.radians(from_north)
        half = np.array([math.sin(t), math.cos(t)]) * self.panel['short'] / 2
        walls['M3'] = self._feet_toward(self._swung(scanned['M3'], centre - half, from_north, self.panel['short']),
                                        (-1., 1.))
        centre = np.asarray(NW_PANEL_CENTRE) + np.array([self.u(-0.1, 0.1), self.u(-0.1, 0.1)])
        yaw = math.radians(NW_PANEL_YAW_DEG + self.u(-4, 4))
        half = np.array([math.cos(yaw), math.sin(yaw)]) * self.panel['long'] / 2
        nw = self._dress(Wall('NW', [(centre - half, centre + half)], math.radians(self.u(-1, 1, False)),
                              self.panel['height'], free_end=centre + half, yaw=yaw,
                              cad_pivot=centre - half, pivot=centre - half))
        walls['NW'] = self._feet_toward(nw, (0., -1.))
        return walls


def wall_meshes(wall, rng, strut=(1.05, 0.42)):
    return ms.stand(wall.name, wall.pieces, wall.height, wall.gap, wall.tarp_side, wall.lean, rng,
                    hole=wall.hole, flaps=wall.flaps, strut=strut)


def guideline(walls, joints):
    """The floor line, moved with the maze walls around it: each vertex
    follows the two nearest dividers (as if taped to them), weighted by
    closeness; the barrier carries it in y. ``walls``: as scanned (the tape
    was not re-laid with the rebuild)."""
    movers = [w for w in walls if w.name in ('M1', 'M2', 'M3')]
    out = []
    for p in GUIDELINE:
        p = np.asarray(p, float)
        shifts, weights = [], []
        for w in movers:
            moved = _rot(w.yaw) @ (p - w.cad_pivot) + w.pivot
            cad_free = w.cad_pivot + _rot(-w.yaw) @ (w.free_end - w.pivot)
            d = _segment_distance(p, w.cad_pivot, cad_free)
            shifts.append(moved - p)
            weights.append(1. / (d + 0.1) ** 2)
        shifts.append(np.array([0., Sampler.barrier_y(joints, p[0]) - BARRIER_Y]))
        weights.append(1. / (abs(p[1] - BARRIER_Y) + 0.1) ** 2)
        order = np.argsort(weights)[::-1][:2]
        w = np.array(weights)[order]
        shift = (np.array(shifts)[order] * w[:, None]).sum(axis=0) / w.sum()
        out.append((p + shift).tolist())
    return out


def _segment_distance(p, a, b):
    d = b - a
    t = float(np.clip((p - a) @ d / max(d @ d, 1e-9), 0., 1.))
    return float(np.linalg.norm(a + t * d - p))


def _clutter(rng, nets):
    """Boxes outside the net: people (some walking), tables, poles, room walls."""
    boxes = []
    for _ in range(7):
        side = rng.choice(['north', 'south', 'east', 'west'])
        # North is the 3 ft aisle to the next course; west stops at the glass.
        along, out = rng.uniform(-3.5, 3.5), rng.uniform(0.3, 0.75) if side == 'north' else rng.uniform(0.4, 2.0)
        size = (0.45, 0.3, rng.uniform(1.55, 1.9))
        boxes.append({'kind': 'person', 'side': str(side), 'along': along, 'out': out, 'size': size,
                      'amp': float(rng.uniform(0., 1.5)), 'omega': float(rng.uniform(0.2, 0.8)),
                      'phase': float(rng.uniform(0, 6.3))})
    for _ in range(4):
        side = rng.choice(['south', 'east', 'west'])
        boxes.append({'kind': 'table', 'side': str(side), 'along': rng.uniform(-3, 3),
                      'out': rng.uniform(0.6, 1.6 if side == 'west' else 2.5),
                      'size': (1.5, 0.75, 0.75), 'amp': 0., 'omega': 0., 'phase': 0.})
    for side in ('north', 'south', 'east'):
        out = rng.uniform(2.5, 4.) + (NEXT_COURSE_GAP + 2 * NET if side == 'north' else 0.)
        boxes.append({'kind': 'room', 'side': side, 'along': 0., 'out': float(out),
                      'size': (24. if side == 'north' else 12., 0.2, 3.5), 'amp': 0., 'omega': 0., 'phase': 0.})
    for b in boxes:
        b['along'], b['out'] = float(b['along']), float(b['out'])
    return boxes


def _line(name, start, end, normal, checkpoint, route=None):
    return {'name': name, 'start': [float(v) for v in start], 'end': [float(v) for v in end],
            'normal': [float(v) for v in normal], 'checkpoint': checkpoint, 'route': route}


def _lines(walls, tunnel, joints, nets):
    by = {w.name: w for w in walls}

    def from_free_end(wall, cad_free, cad_points, cad_normal, extend):
        """Carry a CAD line attached to a wall's free end with that wall."""
        rotation = _rot(wall.yaw)
        points = [wall.free_end + rotation @ (np.asarray(p) - cad_free) for p in cad_points]
        direction = (points[1] - points[0]) / np.linalg.norm(points[1] - points[0])
        return points[0] - direction * extend[0], points[1] + direction * extend[1], rotation @ np.asarray(cad_normal)
    lines = {}
    # CP1: the end of the first corridor, where the floor line turns east,
    # 0.463 m short of the north net (as in the CAD); from the west net to M1.
    # ``walls``: as scanned, like the tape these lines are tied to.
    y = nets['north'] - CP1_FROM_NET
    m1_x = by['M1'].free_end[0]
    lines['first_corridor'] = _line('first_corridor', (-nets['west'] - 0.3, y), (m1_x, y), (0., 1.), 1)
    s, e, n = from_free_end(by['M3'], np.array([0.305, 2.121]), [(0.305, 2.121), (0.305, 3.048)], (1., 0.), (0., 0.4))
    lines['gate_a'] = _line('gate_a', s, e, n, 2)
    south = tunnel['mouths']['south']
    centre, axis = np.array(south['xy']), np.array(south['axis'])
    across = np.array([axis[1], -axis[0]])
    lines['tunnel_exit'] = _line('tunnel_exit', centre - across * 0.74, centre + across * 0.74, axis, 3)
    north_end = max(by['C2a'].pieces[0][0], by['C2a'].pieces[-1][1], key=lambda p: p[1])
    gate_end = np.array([north_end[0], Sampler.barrier_y(joints, north_end[0])])
    direction = (gate_end - north_end) / np.linalg.norm(gate_end - north_end)
    lines['gate_b'] = _line('gate_b', north_end, gate_end + direction * 0.1, (-direction[1], direction[0]), 4,
                            'gate_b')
    if np.dot(lines['gate_b']['normal'], (-1., 0.)) < 0:
        lines['gate_b']['normal'] = [-v for v in lines['gate_b']['normal']]
    c2a = by['C2a']
    a, b = c2a.pieces[0]
    along = (b - a) / np.linalg.norm(b - a)
    (u0, u1), _ = c2a.hole
    normal = np.array([-along[1], along[0]])
    if normal[0] > 0:
        normal = -normal                      # Crossed westward, into the landing area.
    lines['window'] = _line('window', a + along * (u0 + 0.05), a + along * (u1 - 0.05), normal, 4, 'window')
    return lines


def _band_raster(groups, z0, z1, res=0.05, extent=NET + 0.3):
    """Cells of an XY grid touched by any surface within heights z0..z1."""
    n = int(2 * extent / res)
    grid = np.zeros((n, n), bool)
    for vertices, tris in groups.values():
        if len(tris) == 0:
            continue
        tri = vertices[tris]
        tri = tri[(tri[:, :, 2].max(axis=1) >= z0) & (tri[:, :, 2].min(axis=1) <= z1)]
        if not len(tri):
            continue
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        longest = np.maximum.reduce([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1),
                                     np.linalg.norm(c - b, axis=1)])
        steps = np.maximum(1, np.ceil(longest / (res * 0.5))).astype(int)
        for k in np.unique(steps):
            pick = steps == k
            u, v = np.meshgrid(np.arange(k + 1), np.arange(k + 1))
            keep = (u + v) <= k
            u, v = u[keep] / k, v[keep] / k
            pa, pb, pc = a[pick], b[pick], c[pick]
            p = (pa[:, None] + u[None, :, None] * (pb - pa)[:, None]
                 + v[None, :, None] * (pc - pa)[:, None]).reshape(-1, 3)
            p = p[(p[:, 2] >= z0) & (p[:, 2] <= z1)]
            i = np.floor((p[:, :2] + extent) / res).astype(int)
            ok = np.all((i >= 0) & (i < n), axis=1)
            grid[i[ok, 0], i[ok, 1]] = True
    return grid, res, extent


def _reachable(blocked, start, res, extent):
    from collections import deque
    n = blocked.shape[0]
    seen = np.zeros_like(blocked)
    i0 = tuple(np.floor((np.asarray(start) + extent) / res).astype(int))
    if blocked[i0]:
        return seen
    queue = deque([i0])
    seen[i0] = True
    while queue:
        i, j = queue.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < n and 0 <= b < n and not seen[a, b] and not blocked[a, b]:
                seen[a, b] = True
                queue.append((a, b))
    return seen


def _inflate(grid, radius, res):
    from scipy.ndimage import distance_transform_edt
    return distance_transform_edt(~grid) * res <= radius


def validate(course, radius=BODY_RADIUS + 0.05):
    """Reasons the layout is unflyable or unfair (empty when it is fine).
    Every passage must leave the drone ``radius`` - body radius of slack a
    side at flight height, tarps included (organizers build a flyable course)."""
    solid = {k: v for k, v in course.groups.items() if not k.startswith(('Floor', 'Landing'))}
    grid, res, extent = _band_raster(solid, 0.62, 0.95)
    n = grid.shape[0]
    xs = (np.arange(n) + 0.5) * res - extent
    xx, yy = np.meshgrid(xs, xs, indexing='ij')
    nets = course.nets
    outside = (xx < -nets['west']) | (xx > nets['east']) | (yy < -nets['south']) | (yy > nets['north'])
    grid |= outside
    at = lambda p: tuple(np.floor((np.asarray(p) + extent) / res).astype(int))
    mouths = course.tunnel['mouths']
    pre = np.array(mouths['north']['xy']) + np.array(mouths['north']['axis']) * 0.45
    post = np.array(mouths['south']['xy']) + np.array(mouths['south']['axis']) * 0.45
    launch = np.array(course.launch['xy'])
    problems = []
    i = at(launch)
    # Whoever places the drone keeps it visibly clear of the walls (8 cm); the
    # stands' pavers and battens on the floor need only clear its footprint.
    # Floor level: 5 cm (1 cm let the contact judge register a stand's paver at t=0, seeds 5 and 9).
    # And down to the floor, nothing under its body and arms (a swung wall's batten, seed 5).
    for z0, z1, clear in ((0.10, 1.0, BODY_RADIUS + 0.08), (0.03, 0.10, BODY_RADIUS + 0.05),
                          (-0.005, 0.03, LAUNCH_FLOOR_CLEAR)):
        layer, _, _ = _band_raster(solid, z0, z1)
        r = int(math.ceil(clear / res))
        if layer[i[0] - r:i[0] + r + 1, i[1] - r:i[1] + r + 1][
                np.hypot(*np.mgrid[-r:r + 1, -r:r + 1]) * res <= clear].any():
            problems.append('a tarp touches the drone on the launch mark')
            break
    if not getattr(course, 'rocks_placed', True):
        problems.append('a rock found no room')
    for rock in course.boulders:
        if mat_gap(rock['xy'], rock['radius'], course.pad, course.pad_mat) < ROCK_CLEAR - 1e-6:
            problems.append('a rock beside the mat')
    # Nothing through the net, and the mat far enough in to land on.
    inside = np.vstack([v for name, (v, t) in course.groups.items()
                        if name != 'Floor' and not name.startswith('PVC net frame') and len(t)])
    if (inside[:, 0].max() > nets['east'] + 0.005 or -inside[:, 0].min() > nets['west'] + 0.005
            or inside[:, 1].max() > nets['north'] + 0.005 or -inside[:, 1].min() > nets['south'] + 0.005):
        problems.append('something through the net')
    px, py = course.pad[:2]
    if min(nets['east'] - px, nets['west'] + px, nets['north'] - py, nets['south'] + py) < 0.45:
        problems.append('the mat against the net')
    above_floor, _, _ = _band_raster(solid, 0.03, 1.0)
    p = at(course.pad[:2])
    r = int(math.ceil(0.33 / res))      # the drone's landing footprint, with a few cm
    if above_floor[p[0] - r:p[0] + r + 1, p[1] - r:p[1] + r + 1][np.hypot(*np.mgrid[-r:r + 1, -r:r + 1]) * res <= 0.33].any():
        problems.append('something stands on or beside the landing mat')
    fat = _inflate(grid, radius, res)
    # The maze's far gates (M1's end, gate A) are against the north net, which
    # the drone cannot see: they must leave room for where it might be.
    sagged = _inflate(grid | (yy > nets['north'] - NORTH_NET_SLACK), radius, res)
    seen = _reachable(sagged, launch, res, extent)
    if not seen[at(pre)]:
        problems.append('maze closed: launch cannot reach the tunnel mouth')
    if not _reachable(fat, pre, res, extent)[at(post)]:
        problems.append('tunnel blocked')
    pad = np.array(course.pad[:2])
    window, gate_b = course.lines['window'], course.lines['gate_b']

    def blocked_line(line, base=None):
        g = (fat if base is None else base).copy()
        for f in np.linspace(-0.2, 1.2, 60):
            p = np.asarray(line['start']) + f * (np.asarray(line['end']) - np.asarray(line['start']))
            i = at(p)
            g[max(i[0] - 1, 0):i[0] + 2, max(i[1] - 1, 0):i[1] + 2] = True
        return g
    # Through the window: the band inside its opening (high on its wall).
    c2a = next(w for w in course.walls if w.name == 'C2a')
    (_, _), (sill, lintel) = c2a.hole
    high, _, _ = _band_raster(solid, sill + 0.08, min(sill + 0.4, lintel - 0.08))
    snug = _inflate(high | outside, BODY_RADIUS + 0.02, res)      # The real window is that tight.
    if not _reachable(blocked_line(gate_b, snug), post, res, extent)[at(pad)]:
        problems.append('window route closed')
    course.gate_b_open = bool(_reachable(blocked_line(window), post, res, extent)[at(pad)])
    thin = _inflate(grid, 0.12, res)
    for which in ('north', 'south'):
        m = mouths[which]
        centre, axis = np.array(m['xy']), np.array(m['axis'])
        across = np.array([axis[1], -axis[0]])
        for f in np.linspace(-0.8, 0.8, 50):
            thin[at(centre + across * f)] = True
    if _reachable(thin, launch, res, extent)[at(post)]:
        problems.append('a way around the tunnel is open')
    return problems


def generate(seed, severity=1.0, attempts=120, layout=None, ladder=True, ladder_pose=None):
    """A validated irregular course; severity scales every offset. ``layout``:
    one of LAYOUTS (default LAYOUT). ``ladder``: the stepladder added to the
    course on 2026-11-07 (localsim.messy_ladder), at ``ladder_pose`` (a name in
    messy_ladder.POSES; None: its DEFAULT_POSE)."""
    for attempt in range(attempts):
        course = _sample(seed * 1000 + attempt, severity, layout, ladder, ladder_pose)
        if not validate(course):
            course.seed = seed
            course.attempt = attempt
            _surroundings(course)
            return course
    raise RuntimeError(f'no flyable layout for seed {seed} at severity {severity}')


def _surroundings(course):
    """The next net north across the aisle (its PVC frame; the viewer and the
    RGB model lay its blue floor tarp), and the glass wall west."""
    dy = course.nets['north'] + NEXT_COURSE_GAP + course.nets['south']
    for name, (vertices, tris) in list(course.groups.items()):
        if name.startswith('PVC'):
            course.groups[f'{name} (next course)'] = (vertices + np.array([0., dy, 0.]), tris)
    course.hall = {'next_course_dy': float(dy), 'glass_x': float(-course.nets['west'] - GLASS_FROM_NET)}


def _place_pad(sampler, walls, tries=200):
    """The mat: centre anywhere in PAD_AREA, PAD_FROM_WINDOW_WALL off the
    window wall, every corner inside the net; size (m) and yaw (deg), any
    turn (a square mat repeats every 90 degrees)."""
    a, b = (np.asarray(p) for p in next(w for w in walls if w.name == 'C2a').pieces[0])
    size, yaw = sampler.u(0.75, 0.9, False), sampler.u(-45., 45., False)
    corners = (np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * size / 2) @ _rot(math.radians(yaw)).T
    (x0, x1), (y0, y1) = PAD_AREA
    for _ in range(tries):
        centre = np.array([sampler.u(x0, x1, False), sampler.u(y0, y1, False)])
        f = np.clip((centre - a) @ (b - a) / ((b - a) @ (b - a)), 0., 1.)
        if np.linalg.norm(centre - (a + f * (b - a))) >= PAD_FROM_WINDOW_WALL \
                and (np.abs(centre + corners) <= NET - 0.05).all():
            break
    return (float(centre[0]), float(centre[1]), PAD[2]), (size, yaw)


def mat_gap(centre, radius, pad, pad_mat):
    """Floor distance from a rock's edge to the mat's square (side, yaw) edge; < 0 overlaps."""
    side, yaw = pad_mat
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    dx, dy = centre[0] - pad[0], centre[1] - pad[1]
    u, v = abs(c * dx + s * dy) - side / 2, abs(-s * dx + c * dy) - side / 2
    return math.hypot(max(u, 0.), max(v, 0.)) + min(max(u, v), 0.) - radius


def _rock_spot(sampler, k, pad, reach, pad_mat):
    """A rock's radius and centre: B1-B3 within ``reach`` of the full spread
    about their CAD spots, the rest anywhere around the mat, near edge
    ROCK_CLEAR to ROCK_CLEAR + PAD_ROCK_REACH beyond the mat's corner circle
    (the placement check then holds it ROCK_CLEAR off the mat's square)."""
    rng = sampler.rng
    if k < len(BOULDERS):
        x, y = BOULDERS[k]
        return float(rng.uniform(0.2, 0.3)), np.array([x + reach * float(rng.uniform(-0.35, 0.35)),
                                                       y + reach * float(rng.uniform(-0.35, 0.35))])
    radius = float(rng.uniform(0.24, 0.32))                      # large
    half = pad_mat[0] / 2
    angle = sampler.u(-math.pi, math.pi, False)
    edge = sampler.u(half + ROCK_CLEAR, half * math.sqrt(2) + ROCK_CLEAR + PAD_ROCK_REACH, False)
    return radius, np.asarray(pad[:2]) + (edge + radius) * np.array([math.cos(angle), math.sin(angle)])


def _place_boulders(sampler, groups, pad, pad_mat, tries=80, gap=0.05):
    """The rocks, each re-drawn until it stands clear of every wall, stand,
    paver, the tunnel and the other rocks, inside the net, and off the mat.
    A rock that finds no such spot is left out, never placed on the mat.
    Returns them and whether every one found room."""
    rng = sampler.rng
    grid, res, extent = _band_raster(groups, 0.0, 0.6)
    solid = (np.argwhere(grid) + 0.5) * res - extent
    boulders, placed_all = [], True
    for k in range(len(BOULDERS) + PAD_ROCKS):
        for i in range(tries):
            # A rock that cannot stand at its jittered CAD spot (at severity 0 it
            # has none) searches ever further, up to the full spread.
            radius, centre = _rock_spot(sampler, k, pad, max(sampler.s * JITTER, (i + 1) / tries), pad_mat)
            clear = (np.abs(centre) + radius <= NET - 0.1).all() \
                and np.hypot(*(solid - centre).T).min() > radius + gap \
                and all(math.dist(centre, b['xy']) > radius + b['radius'] + gap for b in boulders) \
                and mat_gap(centre, radius, pad, pad_mat) >= ROCK_CLEAR
            if clear:
                break
        placed_all &= bool(clear)
        if not clear:
            continue
        height = float(rng.uniform(0.45, 0.5334))        # CAD: 21 in at most
        groups[f'Boulder {k + 1}'] = mm.rock(tuple(centre), radius, height, rng)
        boulders.append({'xy': [float(c) for c in centre], 'radius': radius, 'height': height})
    return boulders, placed_all


def _sample(seed, severity, layout=None, ladder=True, ladder_pose=None):
    layout = LAYOUT if layout is None else layout
    if layout not in LAYOUTS:
        raise ValueError(f'layout {layout!r}: not one of {LAYOUTS}')
    sampler = Sampler(seed, severity)
    rng = sampler.rng
    groups, tunnel = sampler.tunnel()
    barrier, joints = sampler.barrier(tunnel)
    placed = {}
    for key in WALLS:
        placed[key] = sampler.wall(key, joints, placed)
    scanned = list(placed.values())           # The tape and its lines follow these.
    if layout == 'rebuilt':
        placed = sampler.rebuild(placed, joints)
    if layout == 'rebuilt':
        # Team, 2026-11-08: panel 1 (behind the drone at takeoff) turned: vinyl toward the ladder,
        # its stand's struts and feet toward the launch corridor.
        Sampler._feet_toward(barrier[0], (0., 1.))
    walls = barrier + list(placed.values())
    pad, pad_mat = _place_pad(sampler, walls)
    for wall in walls:
        if wall.name == 'A1':                 # Its tarped face toward the landing area: no stand feet on the mat.
            a, b = (np.asarray(p) for p in wall.pieces[0])
            normal = np.array([-(b - a)[1], (b - a)[0]])
            wall.tarp_side = 1 if (np.asarray(pad[:2]) - (a + b) / 2) @ normal > 0 else -1
        groups.update(wall_meshes(wall, rng, (sampler.panel['strut_top'], sampler.panel['strut_out'])))
    groups.update(mf.centre_column(Sampler.barrier_y(joints, mf.CENTRE_X)))
    ladder_info = None
    if ladder:
        # Its own generator; the rocks placed after it still retry around it, which
        # moves the shared generator on, so a seed's nets, launch and later attempts differ.
        pose = ladder_pose or ml.DEFAULT_POSE
        centre, yaw, clear = ml.place(np.random.default_rng((seed, 0x1add)), severity * JITTER, groups, _band_raster, pose)
        groups.update(ml.ladder(centre, yaw))
        ladder_info = {'pose': pose, 'centre': list(centre), 'yaw_deg': yaw, 'top_z': ml.TOP_Z, 'spread': ml.SPREAD,
                       'base_width': ml.BASE_WIDTH, 'clear': clear}
    boulders, rocks_placed = _place_boulders(sampler, groups, pad, pad_mat)
    groups['Floor'] = (np.array([[-NET, -NET, -0.01], [NET, -NET, -0.01], [NET, NET, -0.01], [-NET, NET, -0.01]]),
                       np.array([[0, 1, 2], [0, 2, 3]]))
    # Each net a few cm off the CAD's, but never inside what stands against it.
    solid = np.vstack([v for name, (v, t) in groups.items() if name != 'Floor' and len(t)])
    reach = {'east': solid[:, 0].max(), 'west': -solid[:, 0].min(), 'north': solid[:, 1].max(), 'south': -solid[:, 1].min()}
    nets = {k: max(NET + sampler.u(-0.08, 0.08, False), float(reach[k]) + 0.02) for k in ('north', 'south', 'east', 'west')}
    groups.update(mf.net_frame(nets))
    # The launch mark is taped in the middle of the first corridor (net to M1),
    # its CAD distance north of the barrier.
    m1 = next(w for w in walls if w.name == 'M1')
    mark_x = (-nets['west'] + m1.pivot[0]) / 2
    mark = [mark_x, Sampler.barrier_y(joints, mark_x) + (LAUNCH[1] - BARRIER_Y)]
    # Placed by hand on the taped mark: a few cm off it, the heading by eye.
    launch = {'xy': [mark[0] + sampler.u(-0.06, 0.06, False), mark[1] + sampler.u(-0.06, 0.06, False)],
              'yaw_deg': 90. + sampler.u(-10, 10, False), 'mark': mark}
    tunnel.pop('place_xy')
    course = Course(seed, severity, groups, walls, tunnel, pad, launch, nets, _clutter(rng, nets),
                    boulders=boulders)
    course.pad_mat = pad_mat
    course.ladder = ladder_info
    course.rocks_placed = rocks_placed
    course.layout = layout
    course.lines = _lines(barrier + scanned, tunnel, joints, nets)
    course.guideline = guideline(scanned, joints)
    for k in (1, 2):                                  # The line turns east at CP1.
        course.guideline[k][1] = nets['north'] - CP1_FROM_NET
    return course


def save(course, directory):
    """Write ``course.obj`` and ``course.json``; returns their paths."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    obj = directory / 'course.obj'
    mm.write_obj(obj, course.groups)
    meta = {'seed': course.seed, 'severity': course.severity, 'attempt': getattr(course, 'attempt', 0),
            'layout': getattr(course, 'layout', None),
            'launch': course.launch, 'pad': {'x': course.pad[0], 'y': course.pad[1], 'radius_m': course.pad[2],
                                             'mat_m': course.pad_mat[0], 'yaw_deg': course.pad_mat[1]},
            'nets': course.nets, 'clutter': course.clutter, 'tunnel': course.tunnel, 'lines': course.lines,
            'boulders': course.boulders, 'ladder': getattr(course, 'ladder', None), 'hall': getattr(course, 'hall', None), 'gate_b_open': getattr(course, 'gate_b_open', None),
            'guideline': {'points': course.guideline, 'width': GUIDELINE_WIDTH},
            'walls': [{'name': w.name, 'pieces': [[list(map(float, a)), list(map(float, b))] for a, b in w.pieces],
                       'height': w.height, 'lean_deg': math.degrees(w.lean), 'gap': w.gap,
                       'tarp_side': w.tarp_side, 'print': w.print} for w in course.walls]}
    path = directory / 'course.json'
    path.write_text(json.dumps(meta, indent=1, default=float))
    return obj, path


def checkpoints(meta, template_lines):
    """The referee's Checkpoint objects moved to this course's lines."""
    out = []
    for line in template_lines:
        moved = meta['lines'][line.name]
        out.append(replace(line, start_xy=tuple(moved['start']), end_xy=tuple(moved['end']),
                           forward_normal_xy=tuple(moved['normal'])))
    return out


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Generate an irregular course')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--severity', type=float, default=1.0)
    parser.add_argument('--out', default=str(ROOT / 'localsim/out/messy'))
    parser.add_argument('--layout', choices=LAYOUTS, default=LAYOUT)
    args = parser.parse_args()
    c = generate(args.seed, args.severity, layout=args.layout)
    print(save(c, args.out), 'attempt', c.attempt)
