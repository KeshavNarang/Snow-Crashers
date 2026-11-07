"""Published-CAD loader, swept-height rasterization, and route provenance.

Uses triangle geometry, not object bounding boxes as collision shapes. The
vertical band covers the vehicle height; XY raster cells are expanded by the
vehicle radius, uncertainty margin and half-cell diagonal. Map axes are CAD
X/Y/Z-up; alignment to real odometry is an operator-reviewed deployment input.
"""
from dataclasses import dataclass
from pathlib import Path
import copy
import hashlib
import json
import math
from itertools import islice
from .planning import GridMap, plan_path, NoPathError, PlanningBudget, PlanningResourceError
from .limits import MAX_GRID_AXIS


@dataclass
class Mesh:
    vertices: list
    groups: dict
    sha256: str

    def bounds(self, name):
        points = [self.vertices[i] for tri in self.groups[name] for i in tri]
        return ([min(p[k] for p in points) for k in range(3)],
                [max(p[k] for p in points) for k in range(3)])


def _triangulate_face(vertices,indices):
    """Fan triangulation is valid only for a planar convex polygon."""
    if len(indices) > 3:
        points = [vertices[i] for i in indices]
        normal = [sum((a[(k + 1) % 3] - b[(k + 1) % 3]) * (a[(k + 2) % 3] + b[(k + 2) % 3])
                      for a,b in zip(points,points[1:] + points[:1])) for k in range(3)]
        norm = math.sqrt(sum(v * v for v in normal))
        if norm < 1e-12:
            raise ValueError("OBJ polygon is degenerate or self-intersecting")
        tolerance = 1e-7 * max(1.,max(math.dist(points[0],p) for p in points))
        if any(abs(sum(normal[k] * (p[k] - points[0][k]) for k in range(3))) / norm > tolerance for p in points):
            raise ValueError("OBJ polygon is nonplanar; triangulate it before import")
        drop = max(range(3),key=lambda k:abs(normal[k]))
        axes = [k for k in range(3) if k != drop]
        projected = [(p[axes[0]],p[axes[1]]) for p in points]
        signs = set()
        for i,a in enumerate(projected):
            b = projected[(i + 1) % len(projected)]
            for c in projected:
                cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
                if abs(cross) > 1e-12:
                    signs.add(cross > 0)
        if len(signs) != 1:
            raise ValueError("OBJ polygon is concave; triangulate it before import")
    return [(indices[0],indices[i],indices[i + 1]) for i in range(1,len(indices) - 1)]


def read_obj(path):
    """OBJ v/f/g reader; triangulates polygon fans; handles negative indices."""
    raw = Path(path).read_bytes()
    vertices = []
    groups = {}
    name = "default"
    for line in raw.decode("utf-8").splitlines():
        p = line.split('#',1)[0].strip().split()
        if not p:
            continue
        if p[0] == "g":
            name = " ".join(p[1:])
            groups.setdefault(name,[])
        elif p[0] == "v":
            if len(p) not in (4,5):
                raise ValueError("OBJ vertex must have XYZ and optional homogeneous W")
            coordinates = tuple(float(v) for v in p[1:])
            if not all(math.isfinite(v) for v in coordinates):
                raise ValueError("Invalid OBJ vertex")
            if len(coordinates) == 4 and coordinates[3] == 0:
                raise ValueError("OBJ homogeneous W must not be zero")
            scale = coordinates[3] if len(coordinates) == 4 else 1.
            xyz = tuple(v / scale for v in coordinates[:3])
            if not all(math.isfinite(v) for v in xyz):
                raise ValueError("OBJ homogeneous coordinates overflow")
            vertices.append(xyz)
        elif p[0] == "f":
            if len(p) < 4:
                raise ValueError("OBJ face must have at least three vertices")
            ids = []
            for value in p[1:]:
                n = int(value.split('/')[0])
                i = n - 1 if n > 0 else len(vertices) + n
                if n == 0 or not 0 <= i < len(vertices):
                    raise ValueError("Invalid OBJ index")
                ids.append(i)
            groups.setdefault(name,[]).extend(_triangulate_face(vertices,ids))
    if not vertices or not any(groups.values()):
        raise ValueError("Empty OBJ mesh")
    return Mesh(vertices,groups,hashlib.sha256(raw).hexdigest())


def _clip_z(poly,z,above):
    if not poly:
        return []
    out = []
    a = poly[-1]
    ain = (a[2] >= z if above else a[2] <= z)
    for b in poly:
        bin_ = (b[2] >= z if above else b[2] <= z)
        if ain != bin_:
            t = (z - a[2]) / (b[2] - a[2])
            out.append(tuple(a[k] + t * (b[k] - a[k]) for k in range(3)))
        if bin_:
            out.append(b)
        a = b
        ain = bin_
    return out


def _distance_segment(p,a,b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    d = dx * dx + dy * dy
    t = max(0.,min(1.,((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d)) if d else 0.
    return math.hypot(p[0] - a[0] - t * dx,p[1] - a[1] - t * dy)


def _inside(p,poly):
    inside = False
    a = poly[-1]
    for b in poly:
        if (a[1] > p[1]) != (b[1] > p[1]):
            if p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]:
                inside = not inside
        a = b
    return inside


def _raster_polygon(grid,poly,padding):
    if not poly:
        return
    minx,miny = min(p[0] for p in poly) - padding,min(p[1] for p in poly) - padding
    maxx,maxy = max(p[0] for p in poly) + padding,max(p[1] for p in poly) + padding
    x0,y0 = grid.world_to_cell((minx,miny))
    x1,y1 = grid.world_to_cell((maxx,maxy))
    edges = list(zip(poly,poly[1:] + poly[:1]))
    for y in range(max(0,y0),min(grid.height - 1,y1) + 1):
        for x in range(max(0,x0),min(grid.width - 1,x1) + 1):
            if (x,y) in grid.occupied:
                continue
            p = grid.cell_center((x,y))
            if _inside(p,poly) or any(_distance_segment(p,a,b) <= padding for a,b in edges):
                grid.occupied.add((x,y))


def _slice_segments(mesh,triangles,z):
    segments = []
    for tri in triangles:
        poly = [mesh.vertices[i] for i in tri]
        cross = []
        for a,b in zip(poly,poly[1:] + poly[:1]):
            if (a[2] <= z < b[2]) or (b[2] <= z < a[2]):
                t = (z - a[2]) / (b[2] - a[2])
                cross.append((a[0] + t * (b[0] - a[0]),a[1] + t * (b[1] - a[1])))
        if len(cross) == 2:
            segments.append(tuple(cross))
    return segments


def _fill_slice(grid,segments):
    """Even-odd solid fill; inner tunnel wall forms the inner (free) contour."""
    for iy in range(grid.height):
        y = grid.cell_center((0,iy))[1]
        xs = []
        for a,b in segments:
            if (a[1] <= y < b[1]) or (b[1] <= y < a[1]):
                xs.append(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
        xs.sort()
        if len(xs) % 2:
            raise ValueError("Mesh slice has an open contour; refusing incomplete obstacle volume")
        for lo,hi in zip(xs[::2],xs[1::2]):
            x0 = grid.world_to_cell((lo,y))[0]
            x1 = grid.world_to_cell((hi,y))[0]
            for ix in range(max(0,x0),min(grid.width - 1,x1) + 1):
                x = grid.cell_center((ix,iy))[0]
                if lo <= x <= hi:
                    grid.occupied.add((ix,iy))


def _connected_surfaces(triangles):
    """Union disconnected solids separately; parity must not cancel overlaps.

    Connected inner and outer tunnel walls remain one shell, preserving holes.
    """
    owners = {}
    parents = list(range(len(triangles)))

    def find(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    for i,triangle in enumerate(triangles):
        for vertex in triangle:
            if vertex in owners:
                parents[find(i)] = find(owners[vertex])
            else:
                owners[vertex] = i
    components = {}
    for i,triangle in enumerate(triangles):
        components.setdefault(find(i),[]).append(triangle)
    return list(components.values())


def build_grid(mesh, *, altitude=.90, half_height=.16, radius=.28,
               margin=.10, resolution=.025, bounds=(-3.048,-3.048,3.048,3.048)):
    """Project obstacle volume intersecting swept vertical band into XY.

    A horizontal slice supplies solid interiors, while slab-clipped surfaces
    supply the boundary swept over all heights (not only sampled heights).
    Text/paint/floor and net roof below/above the band naturally do not collide.
    """
    try:
        bounds = tuple(islice(iter(bounds), 5))
        valid = (len(bounds) == 4 and all(not isinstance(x,bool) and math.isfinite(x)
                                          for x in (altitude,half_height,radius,margin,resolution,*bounds)))
    except (TypeError,OverflowError):
        valid = False
    if not valid or min(half_height,radius,resolution) <= 0 or margin < 0:
        raise ValueError("Invalid geometry parameters")
    xmin,ymin,xmax,ymax = bounds
    if xmin >= xmax or ymin >= ymax:
        raise ValueError("Map bounds must have strictly positive width and height")
    zlo,zhi = altitude - half_height - margin,altitude + half_height + margin
    padding = radius + margin + resolution * math.sqrt(2) / 2
    extents = ((xmax - xmin) / resolution, (ymax - ymin) / resolution)
    if any(not math.isfinite(value) or value > MAX_GRID_AXIS for value in extents):
        raise PlanningResourceError('Raster dimensions exceed finite grid resource budget')
    if not all(math.isfinite(value) for value in (zlo, zhi, padding, padding / resolution)):
        raise PlanningResourceError('Raster footprint/slice arithmetic exceeds finite resource domain')
    grid = GridMap(math.ceil(extents[0]), math.ceil(extents[1]), resolution, (xmin, ymin))
    # Explicit bounds fence; ceil-sized last row/column must not extend the net.
    for iy in range(grid.height):
        for ix in range(grid.width):
            x,y = grid.cell_center((ix,iy))
            if min(x - xmin,xmax - x,y - ymin,ymax - y) <= padding:
                grid.occupied.add((ix,iy))
    if len(grid.occupied) == grid.width * grid.height:
        return grid
    for name,tris in mesh.groups.items():
        if not tris:
            continue
        lo,hi = mesh.bounds(name)
        if hi[2] < zlo or lo[2] > zhi:
            continue
        for tri in tris:
            poly = [mesh.vertices[i] for i in tri]
            poly = _clip_z(_clip_z(poly,zlo,True),zhi,False)
            _raster_polygon(grid,[(p[0],p[1]) for p in poly],padding)
        # Filling at centre and band edges handles thick fully enclosed solids.
        for component in _connected_surfaces(tris):
            for z in (zlo + 1e-8,altitude,zhi - 1e-8):
                _fill_slice(grid,_slice_segments(mesh,component,z))
    return grid


def candidate_route(altitude=.90):
    """Inspectable map-derived staging goals, not claimed official checkpoints.

    Gate B route is deliberately selected; the narrow window is not used.
    Exact official checkpoint planes must be measured/reviewed before flight.
    """
    raw = [("first_corridor",-2.50,2.55,1),
           ("maze_turn_north",-1.36,2.55,None),
           ("maze_turn_south",-1.36,.98,None),
           ("maze_second_south",-.20,.98,None),
           ("maze_second_north",-.20,2.55,None),
           ("maze_exit_gate_a",.85,2.55,2),
           ("tunnel_entry",2.31,1.72,None),
           ("tunnel_exit",2.31,-.80,3),
           ("chicane_south",.65,-1.65,None),
           ("gate_b_east",.60,-.05,None),
           ("gate_b_west",-.40,-.05,4),
           ("landing_approach",-1.45,-1.10,None),
           ("landing_pad",-2.04216,-.79248,5)]
    return [dict(name=n,x=x,y=y,z=altitude,checkpoint=c) for n,x,y,c in raw]


def build_course(path, checkpoint_reference=None, **kwargs):
    path_clearance = kwargs.pop('path_clearance_m',.012)
    if isinstance(path_clearance,bool) or not isinstance(path_clearance,(int,float)) or not math.isfinite(path_clearance) or path_clearance < 0:
        raise ValueError('Course path clearance must be finite and nonnegative')
    mesh = read_obj(path)
    required = ('Launch mark','Landing pad cross','Landing pad ring r30',
                'Tube oval 58 x 72 x 60','M1 maze divider','M2 maze divider',
                'M3 maze divider','C1 chicane wall','C2b chicane wall')
    missing = [name for name in required if not mesh.groups.get(name)]
    if missing:
        raise ValueError(
            'Not the supported course layout: missing named geometry ' + ', '.join(missing))
    launch_lo,launch_hi = mesh.bounds('Launch mark')
    pad_lo,pad_hi = mesh.bounds('Landing pad cross')
    ring_lo,ring_hi = mesh.bounds('Landing pad ring r30')
    start = tuple((launch_lo[i] + launch_hi[i]) / 2 for i in (0,1))
    pad = tuple((pad_lo[i] + pad_hi[i]) / 2 for i in (0,1))
    pad_radius = min(*(pad[i] - ring_lo[i] for i in (0,1)),
                     *(ring_hi[i] - pad[i] for i in (0,1)))
    grid = build_grid(mesh,**kwargs)
    grid.path_clearance_m = path_clearance
    altitude = kwargs.get("altitude",.90)
    route = candidate_route(altitude)
    route[-1].update(x=pad[0],y=pad[1])
    previous = start
    segments = []
    issues = []
    budget = PlanningBudget()
    for goal in route:
        end = (goal["x"],goal["y"])
        try:
            points = plan_path(grid,previous,end,budget=budget)
            if goal['name'] in ('chicane_south','gate_b_east') and any(p[0] < 0. for p in points):
                raise NoPathError(
                    'Global path detours into window/landing side before Gate B; requested route topology infeasible')
            segments.append({"to":goal["name"],"path":[list(p) for p in points],
                             "length_m":sum(math.dist(a,b) for a,b in zip(points,points[1:]))})
        except NoPathError as e:
            issues.append({"to":goal["name"],"error":str(e)})
        previous = end
    course = {"schema_version":1,"purpose":"PRE-EVENT REFERENCE; NOT FLIGHT QUALIFIED",
            "source":{"file":Path(path).name,"sha256":mesh.sha256,"units":"metres",
                      "axes":"CAD X/Y, +Z up; origin floor centre; not yet aligned to odometry"},
            "altitude_m":altitude,"radius_m":kwargs.get("radius",.28),
            "path_clearance_m":path_clearance,"handoff_clearance_m":min(.010,path_clearance),
            "half_height_m":kwargs.get("half_height",.16),"margin_m":kwargs.get("margin",.10),
            "dimensions_are_measured":False,
            "launch":{"x":start[0],"y":start[1]},
            "landing":{"x":pad[0],"y":pad[1],"radius_m":pad_radius},
            "route":route,"planned_segments":segments,"route_feasible":not issues,"issues":issues,
            "grid":grid.to_dict(),"group_bounds":{n:mesh.bounds(n) for n in mesh.groups if mesh.groups[n]},
            "limitations":["Preliminary published layout, not an as-built survey",
                           "Vehicle radius/height are explicit unmeasured assumptions including propellers",
                           "CAD-to-estimator origin/yaw must be measured, not inferred from GPS",
                           "Horizontal slice assumes fixed flight altitude; no return-over-barrier route",
                           "Static map cannot establish current free space, lighting or localization health",
                           "Stage goals are candidates, not official whole-drone checkpoint detection"]}
    course['challenge_route_eligible'] = False
    course['checkpoint_numeric_report'] = None
    course['checkpoint_review_blockers'] = ['No reviewed checkpoint reference supplied']
    if checkpoint_reference is not None:
        from .checkpoints import report_path
        from .checkpoint_contract import load_event_reference
        reference = load_event_reference(course, checkpoint_reference)
        course['checkpoint_reference'] = copy.deepcopy(checkpoint_reference)
        course['checkpoint_review_blockers'] = list(reference.blockers)
        if not issues:
            path_xy = [start]
            for segment in segments:
                path_xy.extend(segment['path'][1:])
            # Unknown registration uncertainty remains a deployment blocker.
            # This conditional nominal geometry audit does not assert zero real error.
            radius = course['radius_m'] + course['margin_m'] + (reference.registration_uncertainty_m or 0.)
            report = report_path(reference.checkpoints, path_xy, radius)
            course['checkpoint_numeric_report'] = report.to_dict()
            course['challenge_route_eligible'] = report.complete and not reference.blockers
    return course


def save_course(course,path):
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    Path(path).write_text(json.dumps(course,indent=2) + "\n",encoding="utf-8")


def write_svg(course,path):
    """Standalone inspectable plan drawing, no simulation or web requests."""
    grid = GridMap.from_dict(course["grid"])
    scale = 100.
    pad = 50
    width = grid.width * grid.resolution * scale + 2 * pad
    height = grid.height * grid.resolution * scale + 2 * pad

    def xy(p):return pad + (p[0] - grid.origin[0]) * scale,height - pad - (p[1] - grid.origin[1]) * scale
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="#f8fafc"/>',
           '<text x="20" y="24" font-family="sans-serif" font-size="16">Published CAD: inflated occupied cells + candidate Gate B route</text>']
    for c in sorted(grid.occupied):
        x,y = xy(grid.cell_center(c))
        s = grid.resolution * scale
        out.append(f'<rect x="{x - s / 2:.2f}" y="{y - s / 2:.2f}" '
                   f'width="{s:.2f}" height="{s:.2f}" fill="#334155"/>')
    for segment in course["planned_segments"]:
        pts = ' '.join(f'{x:.2f},{y:.2f}' for x,y in map(xy,segment["path"]))
        out.append(f'<polyline points="{pts}" fill="none" stroke="#0284c7" stroke-width="2.5"/>')
    for i,g in enumerate(course["route"],1):
        x,y = xy((g['x'],g['y']))
        out.append(f'<circle cx="{x}" cy="{y}" r="5" fill="#16a34a"/>'
                   f'<text x="{x + 6}" y="{y - 6}" font-family="sans-serif" font-size="12">{i}</text>')
    x,y = xy((course['launch']['x'],course['launch']['y']))
    out.append(f'<circle cx="{x}" cy="{y}" r="7" fill="#ea580c"/>')
    out.append('</svg>')
    Path(path).write_text('\n'.join(out),encoding='utf-8')
