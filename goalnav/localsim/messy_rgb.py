"""A coarse colour image from the OAK-D's RGB camera on the irregular course.

Enough appearance for the landing-target detector to be tested honestly:
the orange floor tarp (mottled, pebbles), the black tape line, the blue H mat
(textured with the reference cut from a photo, ``assets/pad_reference.npy``),
plywood, grey and red tarps, the wire tunnel, and the hall behind the net
with people in it, some in blue. Exposure varies per frame and every pixel
has sensor noise. Same ray caster as the depth model; the camera is the
team's mount, pitched 26.7 degrees down (``cad/modded-cam.stl``).
"""
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from localsim.messy_world import MESH, PVC, TARP

ROOT = Path(__file__).resolve().parents[1]
ORANGE = np.array([226, 92, 44], float)
BLUE_TARP = np.array([38, 92, 190], float)      # the next course's floor
PEOPLE = np.array([[40, 70, 160], [35, 35, 40], [150, 40, 40], [60, 110, 70], [30, 60, 150], [200, 200, 205]], float)


@dataclass(frozen=True)
class RGBCamera:
    width: int = 160
    height: int = 120
    hfov_deg: float = 69.          # IMX378 on the OAK-D S2
    offset: tuple = (0.10, 0., 0.)
    pitch_deg: float = 26.7

    @property
    def fx(self):
        return (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    def body_from_optical(self):
        t = math.radians(self.pitch_deg)
        pitch = np.array([[math.cos(t), 0., math.sin(t)], [0., 1., 0.], [-math.sin(t), 0., math.cos(t)]])
        return pitch @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


def _hash(ix, iy, salt):
    h = (ix.astype(np.int64) * 73856093) ^ (iy.astype(np.int64) * 19349663) ^ salt
    return (h % 10007) / 10007.


class FloorPainter:
    def __init__(self, meta):
        self.reference = np.load(ROOT / 'assets/pad_reference.npy').astype(float)
        pad = meta['pad']
        self.pad = np.array([pad['x'], pad['y']])
        self.pad_size = pad.get('mat_m', 0.85)
        self.pad_yaw = math.radians(pad.get('yaw_deg', 0.))
        line = meta.get('guideline') or {}
        self.line = np.array(line.get('points', []), float)
        self.line_half = line.get('width', 0.051) / 2

    def colour(self, xy, base=None):
        x, y = xy[:, 0], xy[:, 1]
        mottle = 0.9 + 0.06 * np.sin(x * 7.3 + np.sin(y * 3.1)) + 0.05 * np.sin(y * 11.7 + x * 2.3)
        rgb = (ORANGE if base is None else base)[None] * mottle[:, None]
        cell = 0.22
        ix, iy = np.floor(x / cell), np.floor(y / cell)
        cx = (ix + 0.2 + 0.6 * _hash(ix, iy, 11)) * cell
        cy = (iy + 0.2 + 0.6 * _hash(ix, iy, 23)) * cell
        pebble = (_hash(ix, iy, 37) < 0.45) & (np.hypot(x - cx, y - cy) < 0.028)
        rgb[pebble] = [95, 45, 25]
        if len(self.line) > 1:
            near = np.full(len(x), np.inf)
            for a, b in zip(self.line, self.line[1:]):
                d = b - a
                t = np.clip(((xy - a) @ d) / (d @ d), 0., 1.)
                near = np.minimum(near, np.linalg.norm(a + t[:, None] * d - xy, axis=1))
            rgb[near < self.line_half] = [22, 22, 22]
        c, s = math.cos(self.pad_yaw), math.sin(self.pad_yaw)
        rel = xy - self.pad
        u = (c * rel[:, 0] + s * rel[:, 1]) / self.pad_size + 0.5
        v = (-s * rel[:, 0] + c * rel[:, 1]) / self.pad_size + 0.5
        on = (u >= 0) & (u < 1) & (v >= 0) & (v < 1)
        n = self.reference.shape[0]
        rgb[on] = self.reference[np.clip(((1 - v[on]) * n).astype(int), 0, n - 1),
                                 np.clip((u[on] * n).astype(int), 0, n - 1)]
        return rgb


def render(world, camera, painter, position, rotation, rng):
    """(height, width, 3) uint8 image."""
    u = np.arange(camera.width) + 0.5 - camera.width / 2
    v = np.arange(camera.height) + 0.5 - camera.height / 2
    uu, vv = np.meshgrid(u, v)
    rays = np.stack([uu / camera.fx, vv / camera.fx, np.ones_like(uu)], axis=-1).reshape(-1, 3)
    directions = rays @ (rotation @ camera.body_from_optical()).T
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    origin = position + rotation @ np.asarray(camera.offset)
    hit = world.cast(origin, directions, 0.05, 12.)
    with np.errstate(divide='ignore', invalid='ignore'):
        floor_t = np.where(directions[:, 2] < 0, -origin[2] / directions[:, 2], np.inf)
    rgb = np.zeros((len(directions), 3))
    inside_net = world.net_low[:2], world.net_high[:2]
    floor_point = origin + directions * np.where(np.isfinite(floor_t), floor_t, 0.)[:, None]
    hit_z = origin[2] + directions[:, 2] * np.where(np.isfinite(hit), hit, 0.)
    grazing = np.isfinite(hit) & (hit_z < 0.03)          # The floor's own voxels, met early at a slant.
    on_floor = np.isfinite(floor_t) & ((floor_t <= hit + 0.03) | grazing) \
        & np.all((floor_point[:, :2] >= inside_net[0]) & (floor_point[:, :2] <= inside_net[1]), axis=1)
    if on_floor.any():
        rgb[on_floor] = painter.colour(floor_point[on_floor, :2])
    hall = world.meta.get('hall')
    if hall:
        # The next net's floor: a plain blue tarp.
        shift = np.array([0., hall['next_course_dy']])
        next_floor = np.isfinite(floor_t) & ((floor_t <= hit + 0.03) | grazing) & ~on_floor \
            & np.all((floor_point[:, :2] - shift >= inside_net[0]) & (floor_point[:, :2] - shift <= inside_net[1]), axis=1)
        if next_floor.any():
            rgb[next_floor] = BLUE_TARP
        on_floor = on_floor | next_floor
    solid = np.isfinite(hit) & ~on_floor
    if solid.any():
        p = origin + directions[solid] * hit[solid, None]
        index = np.clip(np.floor((p - world.origin) / world.voxel).astype(int), 0, world.shape - 1)
        label = world.label[tuple(index.T)]
        shade = 0.85 + 0.3 * _hash(np.floor(p[:, 0] / 0.3), np.floor(p[:, 2] / 0.3), 5)[:, None]
        colour = np.where((label == TARP)[:, None], _tarp_colour(world, p), [208, 182, 138])
        colour = np.where((label == MESH)[:, None], [45, 40, 34], colour)
        colour = np.where((label == PVC)[:, None], [24, 24, 26], colour)
        rgb[solid] = colour * shade
    rest = ~on_floor & ~solid
    if rest.any():
        rgb[rest] = _beyond(world, origin, directions[rest])
    gain = rng.uniform(0.85, 1.15)
    rgb = rgb * gain + rng.normal(0., 6., rgb.shape)
    return np.clip(rgb, 0, 255).astype(np.uint8).reshape(camera.height, camera.width, 3)


def _tarp_colour(world, points):
    """Grey sci-fi print or red rock, by the nearest wall's print."""
    walls = getattr(world, '_tarp_walls', None)
    if walls is None:
        walls = [(np.mean(np.array(w['pieces'][0]), axis=0), w.get('print', 'scifi')) for w in world.meta['walls']]
        walls.append((np.array(world.meta['tunnel']['mouths']['north']['xy']), 'rock'))
        world._tarp_walls = walls
    centres = np.array([c for c, _ in walls])
    rock = np.array([p == 'rock' for _, p in walls])
    nearest = np.argmin(np.linalg.norm(points[:, None, :2] - centres[None], axis=2), axis=1)
    return np.where(rock[nearest][:, None], [112, 46, 30], [186, 189, 194])


def _beyond(world, origin, directions):
    """The hall: clutter boxes (people, some in blue), wood-panelled walls, ceiling."""
    best = np.full(len(directions), np.inf)
    colour = np.zeros((len(directions), 3))
    for k, (low, high) in enumerate(world.clutter_boxes()):
        t = world._box_entry(origin, directions, low, high)
        closer = t < best
        best[closer] = t[closer]
        colour[closer] = PEOPLE[k % len(PEOPLE)]
    far = ~np.isfinite(best)
    z = origin[2] + directions[far, 2] * 6.
    colour[far] = np.where((z < 1.6)[:, None], [120, 80, 48], [228, 205, 170])
    colour[far & (directions[:, 2] > 0.5)] = [60, 55, 50]
    hall = world.meta.get('hall')
    if hall:
        from localsim.messy_world import glass_hit
        t, frame = glass_hit(hall['glass_x'], origin, directions)
        with np.errstate(divide='ignore', invalid='ignore'):
            floor = np.where(directions[:, 2] < 0, -origin[2] / directions[:, 2], np.inf)
        glass = (t < best) & (t < floor)
        colour[glass] = [235, 238, 242]                      # Daylight through the windows.
        colour[glass & frame] = [70, 72, 78]                 # Mullions, sill and head.
    return colour
