"""The route as a chain of goals, each placed relative to the one before.

Nothing is registered to a course origin. The CAD contributes only the
offsets between consecutive goals (``config/goals.yaml``). A goal's position
estimate is, in order of preference:

1. what the live map shows near it: the point of most clearance in a small
   search window around the prior, across the direction of travel (this
   finds a corridor's middle, a gate between two wall ends, a window in a
   wall, the inside of the tunnel);
2. otherwise, the previous goal's estimate plus the CAD offset, rotated by
   the course heading the drone has measured from the walls so far.

So when the drone finds Gate A 0.5 m off, every goal after it moves with
it, and the error left is only the slack between neighbours.
"""
from dataclasses import dataclass, field
import math

import numpy as np

KINDS = ('waypoint', 'gap', 'reach', 'tube', 'opening', 'pad')


@dataclass
class Goal:
    id: str
    kind: str
    cad: tuple                  # nominal pass point (CAD metres)
    heading_deg: float          # direction of travel there (CAD, CCW from +x)
    across: float = 0.3         # search half-width across the travel direction
    along: float = 0.2          # search half-length along it
    checkpoint: str = None
    height: float = None        # pass height above the floor (default: cruise)
    anchor: str = None          # placed from this goal's estimate instead of the previous one
    line_turn: str = None       # the floor line turns 'left'/'right' here: place the goal just past the turn
    turn_to_net: float = None   # the net ahead is this far past that turn (CAD): it anchors that side
    confine: float = None       # plan the leg to this goal only within this far of its straight line
    lookouts: tuple = ()        # pad: CAD points to look for it from, in turn, until it is seen
    axis: np.ndarray = None     # measured crossing direction (openings: the wall's normal)
    detected: np.ndarray = None
    estimate: np.ndarray = None
    confidence: float = 0.
    passed: float = None
    pass_height: float = None
    samples: list = field(default_factory=list)

    @classmethod
    def from_dict(cls, d):
        kind = d.get('kind', 'waypoint')
        if kind not in KINDS:
            raise ValueError(f'goal {d.get("id")}: unknown kind {kind}')
        return cls(id=str(d['id']), kind=kind, cad=tuple(float(v) for v in d['at']),
                   heading_deg=float(d['heading']), across=float(d.get('across', 0.3)),
                   along=float(d.get('along', 0.2)), checkpoint=d.get('checkpoint'),
                   height=None if d.get('height') is None else float(d['height']), anchor=d.get('anchor'),
                   line_turn=d.get('line_turn'), turn_to_net=d.get('turn_to_net'),
                   confine=None if d.get('confine') is None else float(d['confine']),
                   lookouts=tuple(tuple(float(v) for v in p) for p in d.get('lookouts', ())))


def _rot(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


class Route:
    """The goal chain and the course heading estimate."""

    def __init__(self, spec, start_xy, start_yaw):
        self.launch_cad = np.asarray(spec['launch_cad'], float)
        self.launch_heading = math.radians(float(spec.get('launch_heading_deg', 90.)))
        self.goals = [Goal.from_dict(g) for g in spec['goals']]
        self.net_half = np.asarray(spec.get('net_half', (3.048, 3.048)), float)
        self.start_xy = np.asarray(start_xy, float)
        # Odometry heading of CAD +x: the drone believes it faces the CAD launch heading.
        self.base_yaw = float(start_yaw) - self.launch_heading
        self.heading_correction = 0.
        self.index = 0
        self.update_estimates()

    # -- frames ------------------------------------------------------------------
    @property
    def yaw(self):
        return self.base_yaw + self.heading_correction

    def cad_vector(self, v):
        return _rot(self.yaw) @ np.asarray(v, float)

    def cad_point(self, p):
        """Where the CAD puts a point, seen from the launch (prior only)."""
        return self.start_xy + self.cad_vector(np.asarray(p, float) - self.launch_cad)

    def direction(self, goal):
        a = math.radians(goal.heading_deg)
        return self.cad_vector((math.cos(a), math.sin(a)))

    def lookout(self, goal, k):
        """A goal's ``k``-th lookout, carried along with its prior."""
        return goal.prior + self.cad_vector(np.asarray(goal.lookouts[k]) - np.asarray(goal.cad))

    def net_centre(self):
        return self.cad_point((0., 0.))

    # -- estimates -----------------------------------------------------------------
    def update_estimates(self):
        by_id = {g.id: g for g in self.goals}
        previous_cad, previous = self.launch_cad, self.start_xy
        for goal in self.goals:
            prior = previous + self.cad_vector(np.asarray(goal.cad) - previous_cad)
            goal.prior = prior
            if goal.detected is not None and goal.kind not in ('pad', 'opening') and \
                    np.linalg.norm(goal.detected - prior) > max(goal.across, goal.along) + 0.5:
                goal.detected = None              # Found under an earlier, wrong prior: look again.
            goal.estimate = goal.detected if goal.detected is not None else prior
            previous_cad, previous = np.asarray(goal.cad), goal.estimate
        for _ in range(2):                    # An anchor may come later in the chain.
            for goal in self.goals:
                if goal.anchor:
                    a = by_id[goal.anchor]
                    offset = self.cad_vector(np.asarray(goal.cad) - np.asarray(a.cad))
                    if a.axis is not None:          # Straight through, square to the real wall.
                        offset = a.axis * float(offset @ self.direction(a))
                    goal.prior = goal.estimate = a.estimate + offset

    @property
    def current(self):
        return self.goals[self.index] if self.index < len(self.goals) else None

    def upcoming(self, count=3):
        return self.goals[self.index:self.index + count]

    def advance_if_passed(self, xy, now):
        """Pass the current goal when the drone crosses its pass line."""
        goal = self.current
        if goal is None or goal.kind == 'pad':      # The pad ends with a landing, not a line.
            return False
        anchor = next((g for g in self.goals if g.id == goal.anchor), None) if goal.anchor else None
        if anchor is not None and anchor.kind == 'opening' and anchor.detected is None:
            return False                       # In front of a window not found yet: keep looking.
        measured = goal.kind == 'opening' and goal.axis is not None
        d = np.asarray(goal.axis, float) if measured else self.direction(goal)
        offset = np.asarray(xy, float) - goal.estimate
        along, across = float(offset @ d), float(abs(offset @ np.array([-d[1], d[0]])))
        if goal.kind == 'tube':
            through = along >= 0.15 and across <= 0.2      # Actually in it, not beside it.
        elif measured:
            # Along the measured axis, clear of the far side: the CAD heading can be
            # 20 degrees off a turned wall, and across it reads as progress while
            # the drone is still between the jambs (the manoeuvre exits at 0.55 m).
            through = along >= 0.5 and across <= 0.3
        elif goal.kind == 'opening':
            through = along >= 0.15 and across <= 0.45     # Beyond the wall plane: it went through.
        else:
            through = along >= 0. and across <= goal.across + 0.3
            if goal.kind == 'reach':           # Got as far as is safe.
                through = along >= -0.03 and across <= goal.across + 0.15
            elif not goal.checkpoint:          # A checkpoint needs the line actually crossed.
                through = through or np.hypot(along, across) < 0.3
        if through:
            goal.passed = now
            self.index += 1
            return True
        return False

    def update_heading(self, correction, gain=0.3):
        """Blend in a new course-heading correction (radians)."""
        if correction is not None:
            self.heading_correction += gain * (correction - self.heading_correction)


def refine(goal, route, clearance, known, live_map, cap=0.55, min_clearance=0.32, min_known=0.55,
           reach_clearance=0.34):
    """Look for the goal in the live map; returns True when it was found.

    Scans a window around the prior: across the travel direction +-``across``,
    along it +-``along``. The score is the clearance there, saturated at
    ``cap`` (so in open space it stays near the prior) minus a pull back
    toward the prior; only well-observed candidates count.
    """
    d = route.direction(goal)
    normal = np.array([-d[1], d[0]])
    across = np.arange(-goal.across, goal.across + 1e-9, 0.05)
    along = np.arange(-goal.along, goal.along + 1e-9, 0.05 if goal.kind == 'reach' else 0.1)
    a, b = np.meshgrid(across, along)
    centre = goal.prior
    points = centre + a.reshape(-1, 1) * normal + b.reshape(-1, 1) * d
    ij = live_map.cell(points)
    ok = live_map.inside(ij)
    if not ok.any():
        return False
    ij, points, a, b = ij[ok], points[ok], a.reshape(-1)[ok], b.reshape(-1)[ok]
    c = clearance[ij[:, 0], ij[:, 1]]
    k = known[ij[:, 0], ij[:, 1]]
    pull = 0.10 * np.abs(a) / max(goal.across, 1e-6) + 0.05 * np.abs(b) / max(goal.along, 1e-6)
    score = np.minimum(c, cap) - pull
    if goal.kind == 'reach':
        # The end of a dead end (CP1): as far along as stays safely clear.
        score = b + 0.2 * np.minimum(c, cap)
        min_clearance = reach_clearance
    elif goal.kind == 'opening':
        # A window is barely wider than the drone: its centre has only 0.32-0.40 m
        # of clearance even before map error, so look for the best, not a minimum.
        min_clearance = 0.22
    score[(k < min_known) | (c < min_clearance)] = -np.inf
    best = int(np.argmax(score))
    if not np.isfinite(score[best]):
        return False
    found = points[best]
    goal.detected = found if goal.detected is None else goal.detected + 0.5 * (found - goal.detected)
    goal.confidence = min(1., goal.confidence + 0.2)
    return True


def wall_line(xy, point, direction, reach, min_cells, slab=0.25):
    """(centre, along) of the straight wall near ``point``, crossed along
    ``direction``: a line fit to the occupied cells ``xy`` within ``reach``,
    or None if they are not a line. Only cells near the wall's middle depth
    across ``direction`` count: a rock or a stand's foot within reach, a few
    cells against a hundred on the wall, would otherwise turn the fit and
    fail the line test."""
    near = xy[np.linalg.norm(xy - point, axis=1) < reach]
    depth = (near - point) @ direction
    near = near[np.abs(depth - np.median(depth)) < slab] if len(near) else near
    if len(near) < min_cells:
        return None
    centre = near.mean(axis=0)
    _, values, vt = np.linalg.svd(near - centre)
    if values[0] < 2.5 * max(values[1], 1e-6):
        return None                                    # Not a line.
    return centre, vt[0]


def opening_axis(goal, route, live_map, z_low, z_high, reach=0.9):
    """Square an opening to its wall: fit a line to the wall above it (solid
    across the whole width there), put the goal on that line and set its
    crossing axis to the wall normal. Returns True when it could."""
    cells = np.argwhere(live_map.blocked(z_low, z_high))
    fit = wall_line(live_map.centre_of(cells), goal.detected, route.direction(goal), reach, 8)
    if fit is None:
        return False
    centre, along = fit
    normal = np.array([-along[1], along[0]])
    if normal @ route.direction(goal) < 0:
        normal = -normal
    if abs(normal @ route.direction(goal)) < math.cos(math.radians(35)):
        return False                                   # Not the wall we expected.
    goal.detected = goal.detected - normal * float((goal.detected - centre) @ normal)
    goal.axis = normal
    return True


def wall_offset(route, live_map, cad_a, cad_b, z_low, z_high, window=0.35, min_cells=20):
    """How far a straight wall sits from where the CAD (seen from the launch)
    puts it, along the CAD wall's left normal: the median signed distance of
    the occupied cells near it. None until enough of it has been seen."""
    a, b = route.cad_point(cad_a), route.cad_point(cad_b)
    cells = np.argwhere(live_map.blocked(z_low, z_high))
    if len(cells) < min_cells:
        return None
    return near_face(live_map.centre_of(cells), a, b, window, min_cells)


def near_face(xy, a, b, window, min_points):
    """Signed distance (along the left normal of a -> b) of a straight wall's
    face from the line a-b, from points near it (between its ends, within
    ``window`` across); None with fewer than ``min_points``."""
    along = (b - a) / np.linalg.norm(b - a)
    normal = np.array([-along[1], along[0]])
    u, d = (xy - a) @ along, (xy - a) @ normal
    near = (u > 0.) & (u < np.linalg.norm(b - a)) & (np.abs(d) < window)
    if near.sum() < min_points:
        return None
    # Its near face, not the median: struts, battens and tarp bulges stand off
    # the face toward the side the drone sees it from, never the other way.
    seen = d[near]
    return float(np.percentile(seen, 20) if np.median(seen) > 0 else np.percentile(seen, 80))


def wall_rotation(route, live_map, cad_a, cad_b, z_low, z_high, window=0.6, min_cells=25, span=0.3, end=0.2):
    """(angle, weight): how far a long straight wall is turned from where the
    CAD (seen from the launch) puts it, from a line fit to the occupied cells
    near it; None until enough of it has been seen."""
    a, b = route.cad_point(cad_a), route.cad_point(cad_b)
    length = float(np.linalg.norm(b - a))
    along = (b - a) / length
    normal = np.array([-along[1], along[0]])
    cells = np.argwhere(live_map.blocked(z_low, z_high))
    if len(cells) < min_cells:
        return None
    xy = live_map.centre_of(cells)
    u, d = (xy - a) @ along, (xy - a) @ normal
    # Clear of the ends: a stand's post and strut stand off the face 6 cm in
    # from each, and the real ends are up to 0.1 m from the CAD's.
    near = (u > end) & (u < length - end) & (np.abs(d) < window)
    if near.sum() < min_cells or np.ptp(u[near]) < span * length:
        return None                                   # Too little of it, or only one end.
    # The median across it in each 10 cm along it, then a line through those:
    # a fit through every cell is tilted by where more cells pile up (stereo
    # noise spreads further where the wall is seen more obliquely); along
    # the ray that noise is about symmetric, so the median stays on the face.
    u, d = u[near], d[near]
    bins = np.floor(u / 0.1).astype(int)
    faces = [(float(np.mean(u[bins == k])), float(np.median(d[bins == k])))
             for k in np.unique(bins) if np.count_nonzero(bins == k) >= 2]
    if len(faces) < 5 or faces[-1][0] - faces[0][0] < span * length:
        return None
    # Theil-Sen: the median of the pairwise slopes, so a bin or two that
    # caught something else cannot tilt it.
    fu, fd = np.array(faces).T
    i, j = np.triu_indices(len(faces), 1)
    slope = float(np.median((fd[j] - fd[i]) / (fu[j] - fu[i])))
    return math.atan(slope), float(len(near))
