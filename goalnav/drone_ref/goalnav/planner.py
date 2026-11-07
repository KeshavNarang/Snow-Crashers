"""Local planning on the live map: clearance, A*, shortcutting, safe speed.

The planner sees three kinds of space: observed free, observed occupied, and
unknown. Unknown is allowed but costs more (optimistic planning: a corridor
that has not been seen yet is assumed open until the depth camera says
otherwise). Speed is limited by the free space actually observed along the
path, so the drone never outruns what it has seen (``drone_ref.braking``).
"""
from dataclasses import dataclass
import heapq
import math

import numpy as np

from drone_ref.braking import speed_for_clearance


@dataclass
class PlanConfig:
    radius: float = 0.35          # preferred clearance: closer only where a passage is narrower
    hard_radius: float = 0.30     # never planned closer (except to back away from the start)
    wall: float = 0.10            # this close to an occupied cell is never planned through
    escape_speed: float = 0.25    # crawl out when already inside the radius
    tight_radius: float = 0.24    # measured clearance accepted on a detected opening's centre line
    blind_range: float = 0.35     # the camera sees nothing nearer (0.31 m): unseen there is not unknown
    soft: float = 0.60            # clearance below this costs extra
    soft_weight: float = 6.
    unknown_cost: float = 0.6
    coarse: int = 2               # planning cell = map cell x coarse
    max_speed: float = 1.6
    reaction_s: float = 0.25
    braking: float = 2.5
    turn_speed: float = 0.45      # speed through a 90 degree turn
    clear_speed_gain: float = 3.0
    speed_caps: bool = True       # the turn and per-goal-kind caps (braking on seen space always applies)


def clearance(blocked, resolution, cap=1.5):
    """Metres from each cell to the nearest blocked cell (capped). Measured
    centre to centre, which under-reads the distance to the surface inside
    the blocked cell by up to half a cell; a third of a cell is added back."""
    try:
        from scipy.ndimage import distance_transform_edt
        d = distance_transform_edt(~blocked) * resolution
        return np.minimum(np.where(blocked, 0., d + resolution / 3), cap)
    except ImportError:
        distance = np.where(blocked, 0., cap)
        frontier = blocked.copy()
        for k in range(1, int(cap / resolution) + 1):
            grown = frontier.copy()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    grown |= np.roll(np.roll(frontier, dx, 0), dy, 1)
            distance[grown & ~frontier] = k * resolution
            frontier = grown
        return distance


def _coarsen(values, factor, how):
    n = values.shape[0] // factor * factor
    v = values[:n, :n].reshape(n // factor, factor, n // factor, factor)
    return how(v, axis=(1, 3))


class Planner:
    def __init__(self, config=None):
        self.cfg = config or PlanConfig()

    def plan(self, live_map, clear, known, start_xy, goals_xy, tight=(), confine=None, exact=None):
        """A path (list of XY) from the start through each goal in turn, or
        None when no goal is reachable. ``tight``: (centre, axis, reach)
        zones (a measured window) where the planner may pass at the body
        radius plus 1 cm, on the line through the opening's centre.
        ``exact``: per goal, end its leg on the goal itself when the fine map
        clears it (a dead end's goal by the net, whose checkpoint must be
        crossed: the coarse cell's centre can be 0.15 m short)."""
        cfg = self.cfg
        f = cfg.coarse
        res = live_map.res * f
        c = _coarsen(clear, f, np.min)            # the worst of a cell: what the margins see
        c = self._tighten(c, live_map.origin, res, tight)
        best = self._tighten(_coarsen(clear, f, np.max), live_map.origin, res, tight)   # passable at all?
        k = _coarsen(known, f, np.mean)
        # Rising steadily as clearance falls toward the hard limit, so a path
        # keeps to the middle even of a passage narrower than the margin it prefers.
        closeness = np.clip((cfg.soft - c) / (cfg.soft - cfg.hard_radius), 0., 1.)
        cost = 1. + cfg.soft_weight * closeness + 40. * np.clip((cfg.radius - c) / (cfg.radius - cfg.hard_radius), 0., 1.) ** 2 \
            + cfg.unknown_cost * (k < 0.5)
        cost[best < cfg.hard_radius] = math.inf  # Never this close (no part of the cell clears it).
        origin = live_map.origin
        to_cell = lambda p: tuple(np.clip(np.floor((np.asarray(p) - origin) / res).astype(int), 0, c.shape[0] - 1))
        legs, here = [], to_cell(start_xy)
        r = 3
        window = (slice(max(here[0] - r, 0), here[0] + r + 1), slice(max(here[1] - r, 0), here[1] + r + 1))
        near = cost[window]
        near[~np.isfinite(near) & (c[window] >= 0.06)] = 60.     # Back away from where it is; never the wall itself.
        for goal in goals_xy:
            target = self._free_near(c, to_cell(goal))
            if not math.isfinite(cost[target]):
                break
            leg_cost = cost
            limit = confine[len(legs)] if confine else None
            if limit:                  # Through the window, never around its wall.
                leg_cost = _confined(cost, here, target, limit / res)
            leg = _astar(leg_cost, here, target)
            if leg is None:
                break
            legs.append(leg)
            here = target
        path = []
        for k, leg in enumerate(legs):
            xy = [tuple(origin + (np.asarray(p) + 0.5) * res) for p in leg]
            if k == 0:
                xy[0] = tuple(start_xy)
            if exact and exact[k] and len(xy) >= 2 and \
                    self._visible(xy[-2], goals_xy[k], live_map, clear, cfg.hard_radius):
                xy[-1] = tuple(np.asarray(goals_xy[k], float))
            xy = self._centre(xy, live_map, clear)
            leg_xy = self._shortcut(xy, live_map, clear)    # Each leg alone: every goal stays on the path.
            path += leg_xy if not path else leg_xy[1:]
        return path if len(path) >= 2 else None

    def _tighten(self, c, origin, res, tight):
        """Inside a tight zone, count clearance against the tight radius
        instead of the usual one (only along the opening's axis)."""
        if not tight:
            return c
        c = c.copy()
        n = c.shape[0]
        centres = origin + (np.indices((n, n)).transpose(1, 2, 0) + 0.5) * res
        for centre, axis, reach in tight:
            d = centres - np.asarray(centre)
            along = d @ np.asarray(axis)
            across = np.abs(d @ np.array([-axis[1], axis[0]]))
            # Map noise under-reads a 0.7 m window by a few cm; its centre line is
            # where the opening was measured, so trust that line through it.
            zone = (np.abs(along) <= reach) & (across <= 0.11) & (c >= self.cfg.tight_radius)   # >= a planning cell wide
            c[zone] = np.maximum(c[zone], self.cfg.radius)
        return c

    def _free_near(self, c, cell, reach=2):
        if c[cell] >= self.cfg.hard_radius:
            return cell
        best, score = cell, -1.
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                a, b = cell[0] + dx, cell[1] + dy
                if 0 <= a < c.shape[0] and 0 <= b < c.shape[1] and c[a, b] - 0.02 * math.hypot(dx, dy) > score:
                    best, score = (a, b), c[a, b] - 0.02 * math.hypot(dx, dy)
        return best

    def _centre(self, xy, live_map, clear, reach=0.1, step=0.025):
        """Slide each interior point across the path onto the fine map's
        best clearance nearby: the coarse grid's cell centres can sit a few
        cm off the true middle of a pinch, which is all its slack."""
        out = [xy[0]]
        offsets = np.arange(-reach, reach + 1e-9, step)
        for k in range(1, len(xy) - 1):
            a, b = np.asarray(xy[k - 1]), np.asarray(xy[k + 1])
            d = b - a
            if np.linalg.norm(d) < 1e-6:
                out.append(xy[k])
                continue
            n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
            candidates = np.asarray(xy[k]) + offsets[:, None] * n
            ij = live_map.cell(candidates)
            ok = live_map.inside(ij)
            values = np.where(ok, clear[np.clip(ij[:, 0], 0, live_map.n - 1), np.clip(ij[:, 1], 0, live_map.n - 1)], -1.)
            values = values - 0.2 * np.abs(offsets)          # Only move for a real gain.
            out.append(tuple(candidates[int(np.argmax(values))]))
        out.append(xy[-1])
        return out

    def _shortcut(self, xy, live_map, clear):
        need = self.cfg.radius + 0.06
        out, i = [xy[0]], 0
        while i < len(xy) - 1:
            j = len(xy) - 1
            while j > i + 1 and not self._visible(xy[i], xy[j], live_map, clear, need):
                j -= 1
            out.append(xy[j])
            i = j
        return out

    @staticmethod
    def _visible(a, b, live_map, clear, need):
        a, b = np.asarray(a), np.asarray(b)
        n = max(2, int(np.linalg.norm(b - a) / (live_map.res * 0.7)))
        points = a + np.linspace(0., 1., n)[:, None] * (b - a)
        ij = live_map.cell(points)
        if not live_map.inside(ij).all():
            return False
        values = clear[ij[:, 0], ij[:, 1]]
        return bool(np.all(values[1:] >= need))

    def turn_limit(self, path, position, deceleration=1.2):
        """Slow down before a turn, not in it: each corner ahead allows
        sqrt(v_turn^2 + 2 a d) at distance d."""
        cfg = self.cfg
        points = [np.asarray(position, float)] + [np.asarray(p, float) for p in path[1:]]
        limit, travelled = cfg.max_speed, 0.
        for a, b, c in zip(points, points[1:], points[2:]):
            travelled += float(np.linalg.norm(b - a))
            u, v = b - a, c - b
            if np.linalg.norm(u) < 1e-6 or np.linalg.norm(v) < 1e-6:
                continue
            angle = math.acos(float(np.clip(u @ v / np.linalg.norm(u) / np.linalg.norm(v), -1., 1.)))
            corner = cfg.max_speed - (cfg.max_speed - cfg.turn_speed) * min(1., angle / (math.pi / 2))
            limit = min(limit, math.sqrt(corner ** 2 + 2 * deceleration * travelled))
            if travelled > 4.:
                break
        return limit

    def speed_limit(self, path, live_map, clear, known, position, heading_change):
        """The speed the path allows now: observed free distance ahead,
        upcoming turns, and clearance here."""
        cfg = self.cfg
        self._path = path
        free = 0.
        previous = np.asarray(position, float)
        ij = live_map.cell(previous)
        here = clear[ij[0], ij[1]] if live_map.inside(ij) else 0.
        for p in path[1:]:
            p = np.asarray(p, float)
            seg = np.linalg.norm(p - previous)
            steps = max(1, int(seg / live_map.res))
            for t in np.linspace(0., 1., steps + 1)[1:]:
                q = previous + t * (p - previous)
                ij = live_map.cell(q)
                c = clear[ij[0], ij[1]] if live_map.inside(ij) else 0.
                closing = c < cfg.hard_radius and c < here - 0.01      # Moving away from a wall is fine.
                unseen = known[ij[0], ij[1]] < 0.5 and free > cfg.blind_range
                if not live_map.inside(ij) or unseen or closing:
                    return self._cap(free, heading_change, clear, live_map, position), free
                here = min(here, c) if c >= cfg.hard_radius else c
                free += seg / steps
            previous = p
            if free > 4.:
                break
        else:
            free += 1.0          # The path ends in seen free space: its end is a stop, not a wall.
        return self._cap(free, heading_change, clear, live_map, position), free

    def _cap(self, free, heading_change, clear, live_map, position):
        cfg = self.cfg
        v = speed_for_clearance(max(0., free - 0.15), cfg.reaction_s, cfg.braking, cfg.max_speed)
        if cfg.speed_caps:
            turn = min(1., abs(heading_change) / (math.pi / 2))
            v = min(v, cfg.max_speed - (cfg.max_speed - cfg.turn_speed) * turn)
            v = min(v, self.turn_limit(self._path, position))
        ij = live_map.cell(position)
        if live_map.inside(ij):
            here = clear[ij[0], ij[1]]
            v = min(v, 0.3 + cfg.clear_speed_gain * max(0., here - cfg.hard_radius))
            if here < cfg.hard_radius:
                v = max(v, cfg.escape_speed)
        return max(v, 0.)


def _confined(cost, a, b, radius):
    """``cost`` with every cell farther than ``radius`` (cells) from the
    segment a -> b made impassable."""
    n0, n1 = cost.shape
    i, j = np.mgrid[0:n0, 0:n1]
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    t = np.clip(((i - a[0]) * d[0] + (j - a[1]) * d[1]) / max(d @ d, 1e-9), 0., 1.)
    far = np.hypot(i - (a[0] + t * d[0]), j - (a[1] + t * d[1])) > radius
    out = cost.copy()
    out[far] = math.inf
    return out


def _astar(cost, start, goal):
    n0, n1 = cost.shape
    steps = [(1, 0, 1.), (-1, 0, 1.), (0, 1, 1.), (0, -1, 1.),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    weights = cost.tolist()
    best = {start: 0.}
    parent = {}
    heap = [(0., start)]
    h = lambda p: math.hypot(p[0] - goal[0], p[1] - goal[1])
    while heap:
        _, here = heapq.heappop(heap)
        if here == goal:
            path = [here]
            while here in parent:
                here = parent[here]
                path.append(here)
            return path[::-1]
        g = best[here]
        for dx, dy, length in steps:
            a, b = here[0] + dx, here[1] + dy
            if not (0 <= a < n0 and 0 <= b < n1):
                continue
            if dx and dy and (weights[here[0] + dx][here[1]] == math.inf or weights[here[0]][here[1] + dy] == math.inf):
                continue                        # No corner cutting past an impassable cell.
            if weights[a][b] == math.inf:
                continue
            candidate = g + length * (weights[a][b] + weights[here[0]][here[1]]) / 2
            if candidate < best.get((a, b), math.inf):
                best[(a, b)] = candidate
                parent[(a, b)] = here
                heapq.heappush(heap, (candidate + h((a, b)), (a, b)))
    return None
