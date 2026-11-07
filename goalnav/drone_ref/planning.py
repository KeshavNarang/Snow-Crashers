"""Conservative 2-D planning and a 3-D swept braking-volume guard.

Grid occupancy represents the *vehicle centre's forbidden region*, so callers
must inflate geometry once before planning. Unknown/out-of-bounds is occupied.
No ROS, simulator, recorded trajectory or external positioning dependency.
"""
from dataclasses import dataclass, field
import heapq
import math
from numbers import Integral
from typing import Iterable
from itertools import islice
from .braking import stopping_distance
from .limits import (MAX_OBSTACLE_POINTS, MAX_GRID_AXIS, MAX_GRID_CELLS,
                     MAX_CAPSULE_CELLS, MAX_SEARCH_STATES, MAX_SEARCH_QUEUE,
                     MAX_PLANNING_WORK)

XY = tuple[float, float]
Cell = tuple[int, int]


class NoPathError(ValueError):
    pass


class PlanningResourceError(NoPathError):
    """A bounded computation could not establish a path; not geometric proof."""


class PlanningBudget:
    """Deterministic aggregate work allowance, optionally shared across goals.

    Geometry calls reserve an upper bound on cell checks; search operations
    consume fixed work units. This limits work, not elapsed hardware time.
    """

    def __init__(self, max_work=MAX_PLANNING_WORK):
        if (isinstance(max_work, bool) or not isinstance(max_work, Integral)
                or not 1 <= max_work <= MAX_PLANNING_WORK):
            raise ValueError('Planning work budget must be a bounded positive integer')
        self.max_work = int(max_work)
        self.used = 0

    def spend(self, amount):
        if isinstance(amount, bool) or not isinstance(amount, Integral) or amount < 0:
            raise ValueError('Planning work must be a nonnegative integer')
        if amount > self.max_work - self.used:
            raise PlanningResourceError('Planning resource budget exhausted; path not established')
        self.used += amount


def _finite(values):
    try:
        return all(not isinstance(v, bool) and math.isfinite(v) for v in values)
    except (TypeError, OverflowError):
        return False


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin: XY = (0.0, 0.0)
    occupied: set[Cell] = field(default_factory=set)
    dynamic: set[Cell] = field(default_factory=set)
    path_clearance_m: float = 0.0

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, Integral) or v <= 0
               for v in (self.width, self.height)):
            raise ValueError("Grid dimensions must be positive integers")
        self.width, self.height = int(self.width), int(self.height)
        if (max(self.width, self.height) > MAX_GRID_AXIS
                or self.width * self.height > MAX_GRID_CELLS):
            raise PlanningResourceError('Grid dimensions exceed the supported resource budget')
        if not _finite((self.resolution,)) or self.resolution <= 0:
            raise ValueError("Grid dimensions and resolution must be positive")
        if not _finite((self.path_clearance_m,)) or self.path_clearance_m < 0:
            raise ValueError('Default path clearance must be finite and nonnegative')
        try:
            self.origin = tuple(islice(iter(self.origin), 3))
        except TypeError as exc:
            raise ValueError("Grid origin must contain two finite numbers") from exc
        if len(self.origin) != 2 or not _finite(self.origin):
            raise ValueError("Grid origin must contain two finite numbers")
        try:
            far = (self.origin[0] + self.width * self.resolution,
                   self.origin[1] + self.height * self.resolution)
        except OverflowError as exc:
            raise ValueError('Grid extent is not representable') from exc
        if not _finite(far) or any(not self.origin[i] < far[i]
                                   or self.origin[i] + self.resolution / 2 == self.origin[i]
                                   or far[i] - self.resolution / 2 == far[i] for i in (0,1)):
            raise ValueError('Grid cells and extent must be representable in floating point')
        self.occupied = self._validated_cells(self.occupied)
        self.dynamic = self._validated_cells(self.dynamic)

    def _validated_cells(self, values):
        """Reject malformed occupancy instead of silently losing obstacles."""
        result = set()
        try:
            count_limit = self.width * self.height
            for index, value in enumerate(islice(iter(values), count_limit + 1)):
                if index >= count_limit:
                    raise PlanningResourceError('Occupancy entry count exceeds grid resource budget, including duplicates')
                cell = tuple(islice(iter(value), 3))
                if len(cell) != 2 or any(isinstance(v, bool) or not isinstance(v, Integral) for v in cell):
                    raise ValueError("Occupancy cells must be integer XY pairs")
                if not (0 <= cell[0] < self.width and 0 <= cell[1] < self.height):
                    raise ValueError("Occupancy cell is outside declared grid bounds")
                result.add((int(cell[0]), int(cell[1])))
        except TypeError as exc:
            raise ValueError("Occupancy must be a collection of integer XY cells") from exc
        return result

    def _grid_coordinates(self, point: XY) -> XY:
        try:
            valid = len(point) == 2 and _finite(point)
        except TypeError:
            valid = False
        if not valid:
            raise ValueError("Point must contain exactly two finite numbers")
        coordinates = ((point[0] - self.origin[0]) / self.resolution,
                       (point[1] - self.origin[1]) / self.resolution)
        if not _finite(coordinates):
            raise ValueError('Point cannot be represented in grid coordinates')
        return coordinates

    def world_to_cell(self, point: XY) -> Cell:
        x,y = self._grid_coordinates(point)
        return math.floor(x),math.floor(y)

    def cell_center(self, cell: Cell) -> XY:
        return (self.origin[0] + (cell[0] + .5) * self.resolution,
                self.origin[1] + (cell[1] + .5) * self.resolution)

    def blocked(self, cell: Cell) -> bool:
        x, y = cell
        return (not (0 <= x < self.width and 0 <= y < self.height)
                or cell in self.occupied or cell in self.dynamic)

    def blocked_xy(self, point: XY) -> bool:
        try:
            return self.blocked(self.world_to_cell(point))
        except ValueError:
            return True

    def segment_free(self, start: XY, end: XY) -> bool:
        """Supercover DDA. Crossing a cell corner checks both adjoining cells.

        Unlike fixed-step sampling, thin occupied cells cannot fall between
        samples. Collinear grid-edge travel checks cells on both sides.
        """
        try:
            x0,y0 = self._grid_coordinates(start)
            x1,y1 = self._grid_coordinates(end)
        except ValueError:
            return False
        x, y = math.floor(x0), math.floor(y0)
        tx, ty = math.floor(x1), math.floor(y1)
        dx, dy = x1 - x0, y1 - y0
        # Endpoints on an edge/corner touch cells on *both* sides, including
        # cells opposite the travel direction. DDA alone misses that contact.

        def endpoint_clear(px, py):
            ix, iy = math.floor(px), math.floor(py)
            # A point rounded just below an edge still needs that edge's two
            # neighbors, not floor(point) and an unrelated extra lower cell.
            bx, by = round(px), round(py)
            xs = (bx, bx - 1) if abs(px - bx) < 1e-12 else (ix,)
            ys = (by, by - 1) if abs(py - by) < 1e-12 else (iy,)
            return all(not self.blocked((cx, cy)) for cx in xs for cy in ys)
        if not endpoint_clear(x0, y0) or not endpoint_clear(x1, y1):
            return False
        edge_x = abs(dx) < 1e-12 and abs(x0 - round(x0)) < 1e-12
        edge_y = abs(dy) < 1e-12 and abs(y0 - round(y0)) < 1e-12

        def okay(cx, cy):
            return not (self.blocked((cx, cy))
                        or (edge_x and self.blocked((cx - 1, cy)))
                        or (edge_y and self.blocked((cx, cy - 1))))
        if not okay(x, y) or not okay(tx, ty):
            return False
        sx = 1 if dx > 0 else -1
        sy = 1 if dy > 0 else -1
        dtx = abs(1 / dx) if dx else math.inf
        dty = abs(1 / dy) if dy else math.inf
        next_x = ((x + 1 - x0) / dx if dx > 0 else (x - x0) / dx) if dx else math.inf
        next_y = ((y + 1 - y0) / dy if dy > 0 else (y - y0) / dy) if dy else math.inf
        for _ in range(self.width + self.height + 4):
            if (x, y) == (tx, ty):
                return True
            # A mixed-direction corner at t=1 can step into the other closed
            # endpoint cell rather than floor(end). Do not keep traversing the
            # infinite supporting line while waiting for that unreachable cell.
            # Endpoint neighbors were checked above, and every t<=1 crossing
            # (including both corner neighbors) retains the normal checks below.
            if min(next_x, next_y) > 1.:
                return True
            if abs(next_x - next_y) < 1e-12:
                if not okay(x + sx, y) or not okay(x, y + sy):
                    return False
                x += sx
                y += sy
                next_x += dtx
                next_y += dty
            elif next_x < next_y:
                x += sx
                next_x += dtx
            else:
                y += sy
                next_y += dty
            if not okay(x, y):
                return False
        return False

    def segment_clearance_free(self, start: XY, end: XY, clearance_m: float) -> bool:
        """Check a continuous XY capsule against closed occupied cell squares.

        This is additional path slack inside the already-inflated free space;
        it never changes the vehicle footprint or clears an occupied cell.
        Segment-to-rectangle distance is exact up to floating point: closest
        points are either a segment endpoint or a rectangle vertex, unless the
        segment intersects the rectangle (already rejected by supercover).
        """
        if not _finite((clearance_m,)) or clearance_m < 0:
            raise ValueError('Path clearance must be finite and nonnegative')
        if not self.segment_free(start, end):
            return False
        if clearance_m == 0:
            return True
        # Work in cell units: squaring metre coordinates can overflow even
        # when the geometry is representable (e.g. a map scaled by 1e200).
        a,b = self._grid_coordinates(start),self._grid_coordinates(end)
        radius = clearance_m / self.resolution
        if not math.isfinite(radius):
            raise PlanningResourceError('Capsule query exceeds finite resource domain')
        xmin,ymin = min(a[0],b[0]) - radius,min(a[1],b[1]) - radius
        xmax,ymax = max(a[0],b[0]) + radius,max(a[1],b[1]) + radius
        coordinate_tolerance = 64 * math.ulp(max(1.,*(abs(v) for v in (*a,*b))))
        if (xmin <= coordinate_tolerance or ymin <= coordinate_tolerance
                or xmax >= self.width - coordinate_tolerance
                or ymax >= self.height - coordinate_tolerance):
            return False
        lo,hi = (math.floor(xmin),math.floor(ymin)),(math.floor(xmax),math.floor(ymax))
        if (hi[0] - lo[0] + 3) * (hi[1] - lo[1] + 3) > MAX_CAPSULE_CELLS:
            raise PlanningResourceError('Capsule query exceeds cell-check resource budget')
        vx,vy = b[0] - a[0],b[1] - a[1]
        length_sq = vx * vx + vy * vy

        def point_segment_distance_sq(x,y):
            projection = ((x - a[0]) * vx + (y - a[1]) * vy) / length_sq if length_sq else 0.
            t = max(0.,min(1.,projection))
            return (x - a[0] - t * vx)**2 + (y - a[1] - t * vy)**2
        radius_sq = radius * radius
        tolerance = coordinate_tolerance * max(1.,radius)
        # Pad both sides: a tangent bounding coordinate can round just below
        # an integer. The distance test below decides actual contact.
        for iy in range(lo[1] - 1,hi[1] + 2):
            for ix in range(lo[0] - 1,hi[0] + 2):
                if not self.blocked((ix,iy)):
                    continue
                x0,y0,x1,y1 = ix,iy,ix + 1,iy + 1
                distances = [point_segment_distance_sq(x,y)
                             for x,y in ((x0,y0),(x0,y1),(x1,y0),(x1,y1))]
                for x,y in (a,b):
                    dx,dy = max(x0 - x,0.,x - x1),max(y0 - y,0.,y - y1)
                    distances.append(dx * dx + dy * dy)
                if min(distances) <= radius_sq + tolerance:
                    return False
        return True

    def add_dynamic_points(self, points_xy: Iterable[XY], radius: float = 0.0):
        """Union observations; cannot erase static geometry.

        Caller controls observation lifetime by clear_dynamic/rebuilding from a
        fresh cloud. `radius` includes vehicle footprint and uncertainty.
        """
        if not _finite((radius,)) or radius < 0:
            raise ValueError("Invalid inflation radius")
        reach = radius + self.resolution * math.sqrt(2) / 2
        if not math.isfinite(reach) or not math.isfinite(reach / self.resolution):
            raise PlanningResourceError('Dynamic inflation exceeds finite resource domain')
        n = math.ceil(reach / self.resolution)
        pending = set()
        checked = 0
        try:
            for index, raw_point in enumerate(islice(iter(points_xy), MAX_OBSTACLE_POINTS + 1)):
                if index >= MAX_OBSTACLE_POINTS:
                    raise PlanningResourceError('Dynamic point count exceeds resource budget')
                point = tuple(islice(iter(raw_point), 3))
                if len(point) != 2 or not _finite(point):
                    continue
                cx, cy = self.world_to_cell(point)
                xlo, xhi = max(0, cx - n), min(self.width - 1, cx + n)
                ylo, yhi = max(0, cy - n), min(self.height - 1, cy + n)
                work = max(0, xhi - xlo + 1) * max(0, yhi - ylo + 1)
                if work > MAX_GRID_CELLS - checked:
                    raise PlanningResourceError('Dynamic inflation cell-check resource budget exhausted')
                checked += work
                for iy in range(ylo, yhi + 1):
                    for ix in range(xlo, xhi + 1):
                        if math.dist(self.cell_center((ix, iy)), point) <= reach:
                            pending.add((ix, iy))
        except TypeError as exc:
            raise ValueError('Dynamic observations must contain bounded XY sequences') from exc
        # Refusal is atomic: no partial observation is reported as complete.
        self.dynamic.update(pending)

    def clear_dynamic(self):
        self.dynamic.clear()

    def to_dict(self):
        return {"width": self.width, "height": self.height,
                "resolution": self.resolution, "origin": list(self.origin),
                "path_clearance_m": self.path_clearance_m,
                "occupied": [list(c) for c in sorted(self.occupied)]}

    @classmethod
    def from_dict(cls, data):
        try:
            return cls(data["width"], data["height"], data["resolution"],
                       data["origin"], data["occupied"],
                       path_clearance_m=data.get('path_clearance_m',0.))
        except (KeyError, TypeError) as exc:
            raise ValueError("Malformed grid configuration") from exc


def plan_path(grid: GridMap, start_xy: XY, goal_xy: XY, *, clearance_m: float | None = None,
              budget: PlanningBudget | None = None) -> list[XY]:
    """A* with no corner cutting and continuous-clearance simplification.

    `clearance_m` adds path slack within existing free cells. Omission uses
    grid.path_clearance_m (zero for legacy maps without the field). It is not a
    replacement for obstacle inflation and cannot reduce the map's margin.
    """
    if clearance_m is None:
        clearance_m = grid.path_clearance_m
    if not _finite((clearance_m,)) or clearance_m < 0:
        raise ValueError('Path clearance must be finite and nonnegative')
    budget = PlanningBudget() if budget is None else budget
    if not isinstance(budget, PlanningBudget):
        raise ValueError('Use a bounded PlanningBudget')

    def clear(a, b):
        first, last = grid._grid_coordinates(a), grid._grid_coordinates(b)
        steps = sum(abs(math.floor(first[i]) - math.floor(last[i])) for i in (0, 1)) + 4
        work = 9 * steps + 16  # Includes both neighbors at every corner.
        radius = clearance_m / grid.resolution
        if radius and math.isfinite(radius):
            bounds = [(min(first[i], last[i]) - radius, max(first[i], last[i]) + radius)
                      for i in (0, 1)]
            if all(math.isfinite(value) for pair in bounds for value in pair):
                scan = math.prod(math.floor(high) - math.floor(low) + 3 for low, high in bounds)
                work += min(scan, MAX_CAPSULE_CELLS + 1)
        budget.spend(work)
        return grid.segment_clearance_free(a, b, clearance_m)
    start, goal = grid.world_to_cell(start_xy), grid.world_to_cell(goal_xy)
    if grid.blocked(start) or grid.blocked(goal):
        raise NoPathError("Start or goal lies in occupied/unknown space")
    if not clear(start_xy,start_xy) or not clear(goal_xy,goal_xy):
        raise NoPathError('Start or goal does not have the requested path clearance')
    if clear(start_xy, goal_xy):
        return [tuple(start_xy), tuple(goal_xy)]

    def heuristic(c):
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
    queue = [(heuristic(start), 0., start)]
    cost = {start: 0.}
    parent = {}
    closed = set()
    while queue:
        budget.spend(1)
        _, g, c = heapq.heappop(queue)
        if c in closed:
            continue
        if c == goal:
            chain = [c]
            while c != start:
                c = parent[c]
                chain.append(c)
            chain.reverse()
            path = [tuple(start_xy)] + [grid.cell_center(q) for q in chain] + [tuple(goal_xy)]
            simplified = [path[0]]
            i = 0
            while i < len(path) - 1:
                j = len(path) - 1
                while j > i and not clear(path[i], path[j]):
                    j -= 1
                if j == i:
                    raise NoPathError("Path endpoint cannot connect safely to grid")
                simplified.append(path[j])
                i = j
            return simplified
        closed.add(c)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            budget.spend(4)
            q = c[0] + dx, c[1] + dy
            if grid.blocked(q):
                continue
            if dx and dy and (grid.blocked((c[0] + dx, c[1])) or grid.blocked((c[0], c[1] + dy))):
                continue
            # Below half a cell, the usual no-corner-cutting stencil already
            # encloses the capsule joining adjacent free cell centres. Larger
            # slack requires checking the actual neighbouring edge too.
            if clearance_m >= grid.resolution / 2 and not clear(grid.cell_center(c),grid.cell_center(q)):
                continue
            ng = g + math.hypot(dx, dy)
            if ng < cost.get(q, math.inf):
                if q not in cost and len(cost) >= MAX_SEARCH_STATES:
                    raise PlanningResourceError('A* state resource budget exhausted; path not established')
                if len(queue) >= MAX_SEARCH_QUEUE:
                    raise PlanningResourceError('A* queue resource budget exhausted; path not established')
                cost[q] = ng
                parent[q] = c
                heapq.heappush(queue, (ng + heuristic(q), ng, q))
    raise NoPathError("No collision-free path in current inflated map")


def guard_velocity(velocity_body, obstacle_points_body, radius=.30,
                   half_height=.16, margin=.08, deceleration=.8,
                   latency=.2, valid=True):
    """Scale desired velocity until swept stopping ellipsoid is obstacle-free.

    Points and velocity are body FLU. This guards a cylinder-like ellipsoid of
    radius+margin and half_height+margin along the commanded displacement.
    Braking distance = v*latency + v^2/(2*a). All observed points matter, not
    just the nearest forward point. A contact at rest suppresses all motion.
    Unknown sensor coverage must be represented by valid=False by the caller.
    This legacy conditional geometry limiter is not used by the production
    adapter and is not a validated aircraft braking model. Arithmetic or
    resource uncertainty suppresses motion.
    """
    try:
        velocity_body = tuple(islice(iter(velocity_body), 4))
    except TypeError:
        return (0., 0., 0.)
    if len(velocity_body) != 3:
        return (0., 0., 0.)
    values = (*velocity_body, radius, half_height, margin, deceleration, latency)
    if not valid or not _finite(values) or min(radius, half_height, deceleration) <= 0 or min(margin, latency) < 0:
        return (0., 0., 0.)
    velocity_body = tuple(map(float, velocity_body))
    speed = math.hypot(*velocity_body)
    if not math.isfinite(speed) or speed == 0.:
        return (0., 0., 0.)
    try:
        points = [tuple(islice(iter(p), 4))
                  for p in islice(iter(obstacle_points_body), MAX_OBSTACLE_POINTS + 1)]
    except TypeError:
        return (0., 0., 0.)
    if len(points) > MAX_OBSTACLE_POINTS or any(len(p) != 3 or not _finite(p) for p in points):
        return (0., 0., 0.)
    # No usable returns is unknown, never proof of free space.
    if not points:
        return (0., 0., 0.)
    r = radius + margin
    h = half_height + margin
    if not math.isfinite(r) or not math.isfinite(h):
        return (0., 0., 0.)

    def clear(scale):
        s = speed * scale
        try:
            travel = stopping_distance(s, latency, deceleration)
        except ValueError:
            return False
        end = tuple(v / speed * travel for v in velocity_body)
        e = (end[0] / r, end[1] / r, end[2] / h)
        length = math.hypot(*e)
        if not math.isfinite(length):
            return False
        direction = tuple(v / length for v in e) if length else (0., 0., 0.)
        for p in points:
            q = (p[0] / r, p[1] / r, p[2] / h)
            if not _finite(q):
                return False
            along = sum(a * b for a, b in zip(q, direction))
            if not math.isfinite(along):
                return False
            along = max(0., min(length, along))
            distance = math.hypot(*(q[i] - along * direction[i] for i in range(3)))
            if not math.isfinite(distance) or distance <= 1.:
                return False
        return True
    if not clear(0.):
        return (0., 0., 0.)
    if clear(1.):
        return tuple(velocity_body)
    lo, hi = 0., 1.
    for _ in range(24):
        mid = (lo + hi) / 2
        if clear(mid):
            lo = mid
        else:
            hi = mid
    return tuple(v * lo * .98 for v in velocity_body)
