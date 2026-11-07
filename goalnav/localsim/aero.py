"""Rough aerodynamic effects for the local simulator, shared by both vehicle models.

Each effect is a cheap closed-form estimate with one gain that turns it off at
zero; ``aero_gain`` scales them all (the ``aero_off`` / ``aero_x2`` suite
rows). Sources and assumptions, for a 1.3 kg quad with 7 inch (0.089 m radius)
props, arms about 0.15 m:

- Ground effect: Cheeseman and Bennett (1955), T/T_inf = 1 / (1 - (R / 4z)^2)
  for rotor height z over the floor, capped (the formula diverges below
  z = R/4 and over-predicts below z = R/2 for multirotors; measured gains there
  are 10-25 %, e.g. Sanchez-Cuevas et al. 2017).
- Rotor drag (blade flapping and induced drag): a horizontal force linear in
  the airspeed across the rotor disc and proportional to thrust (Mahony,
  Kumar and Corke 2012; Faessler et al. 2018 identify 0.2-0.4 1/s at hover
  for 0.5-1 kg quads). In addition to the vehicle's quadratic body drag.
- Wall effect: a rotor near a vertical wall is drawn toward it by a few per
  cent of its thrust within about one rotor diameter (asymmetric inflow;
  McKinnon and Schoellig 2016, Robinson et al. 2014 report 2-5 %). Nets are
  porous and ignored; the floor is the ground effect.
- Tunnel: inside the tube the downwash recirculates; modelled as a small
  thrust change (sign and size uncertain; re-ingested wash lowers it) and
  stronger turbulence.
- Turbulence: the prop wash recirculating in the netted enclosure, a seeded
  first-order Gauss-Markov acceleration per axis, stronger near walls and in
  the tube.

Distances come from a Euclidean distance transform of the course walls
computed once (and cached); each step does one grid lookup and a gradient.
"""
from dataclasses import dataclass
import hashlib
from pathlib import Path

import numpy as np

GRAVITY = 9.81


@dataclass
class AeroConfig:
    """Mixed into VehicleConfig; every gain is zero-able."""
    aero_gain: float = 1.0            # Scales every effect below (0 = all off).
    prop_radius_m: float = 0.089      # 7 inch props.
    rotor_plane_height: float = 0.03  # Rotor plane above the body origin.
    rotor_reach_m: float = 0.24       # Centre to prop tip (0.15 m arm + radius).
    ground_effect: float = 1.0        # Gain on the Cheeseman-Bennett thrust increase.
    ground_effect_max: float = 1.2    # Cap on the thrust ratio.
    rotor_drag_per_s: float = 0.25    # Linear rotor drag at hover thrust, 1/s.
    wall_suction: float = 0.03        # Pull toward a wall at zero gap, fraction of thrust.
    wall_range_m: float = 0.18        # Gap (tip to wall) where the pull fades out: one diameter.
    wall_turbulence: float = 2.0      # Extra turbulence factor at zero gap.
    tunnel_thrust_change: float = -0.03   # Fractional thrust change inside the tube.
    tunnel_turbulence: float = 2.0    # Extra turbulence factor inside the tube.
    turbulence_accel: float = 0.15    # RMS acceleration per axis in open air, m/s^2.
    turbulence_time_s: float = 0.3    # Correlation time.
    ground_friction: float = 0.5      # Coulomb friction after a disarmed drop (not aero_gain scaled).


class WallField:
    """Distance to the nearest non-net wall on a coarse grid, built once."""

    def __init__(self, world, obj_path, cache_dir=None, voxel=0.05, cap=0.6):
        factor = max(1, round(voxel / world.voxel))
        self.voxel, self.origin, self.cap = world.voxel * factor, world.origin, cap
        digest = hashlib.sha256(Path(obj_path).read_bytes()).hexdigest()
        key = hashlib.sha256(f'{digest}:{world.contact.shape}:{factor}:{cap}'.encode()).hexdigest()[:16]
        cache = Path(cache_dir) / f'walls-{key}.npz' if cache_dir else None
        if cache and cache.exists():
            self.distance = np.load(cache)['distance']
        else:
            self.distance = self._build(world, obj_path, factor)
            if cache:
                cache.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(cache, distance=self.distance)
        self.shape = np.array(self.distance.shape)

    @classmethod
    def try_load(cls, world, obj_path, cache_dir=None):
        """The field, or None (no wall effects) when SciPy is missing and nothing is cached."""
        try:
            return cls(world, obj_path, cache_dir)
        except ImportError:
            print('localsim.aero: SciPy not installed, wall effects off')
            return None

    def _build(self, world, obj_path, factor):
        from scipy.ndimage import distance_transform_edt
        from drone_ref.course import read_obj
        walls = world.contact.copy()
        mesh = read_obj(obj_path)
        vertices = np.asarray(mesh.vertices, dtype=float)
        for name, tris in mesh.groups.items():
            if tris and name.startswith('Net'):
                cells = world._surface_cells(vertices[np.asarray(tris)], world.origin, world.shape)
                walls[tuple(cells.T)] = False
        f = factor
        padded = np.zeros(tuple(-(-n // f) * f for n in walls.shape), dtype=bool)
        padded[:walls.shape[0], :walls.shape[1], :walls.shape[2]] = walls
        coarse = padded.reshape(padded.shape[0] // f, f, padded.shape[1] // f, f,
                                padded.shape[2] // f, f).any(axis=(1, 3, 5))
        distance = distance_transform_edt(~coarse) * self.voxel
        return np.minimum(distance, self.cap).astype(np.float32)

    def lookup(self, position):
        """(distance, unit vector away from the wall) at a point; cheap."""
        index = np.floor((position - self.origin) / self.voxel).astype(int)
        if np.any(index < 1) or np.any(index >= self.shape - 1):
            return self.cap, np.zeros(3)
        i, j, k = index
        d = self.distance
        distance = float(d[i, j, k])
        if distance >= self.cap:
            return distance, np.zeros(3)
        gradient = np.array([d[i + 1, j, k] - d[i - 1, j, k], d[i, j + 1, k] - d[i, j - 1, k],
                             d[i, j, k + 1] - d[i, j, k - 1]], dtype=float)
        norm = np.linalg.norm(gradient)
        return distance, (gradient / norm if norm > 1e-9 else np.zeros(3))


class Aero:
    """Per-step aerodynamic thrust factor and extra acceleration."""

    def __init__(self, cfg, world=None, walls=None, seed=0, floor_z=0.):
        self.cfg, self.world, self.walls, self.floor_z = cfg, world, walls, floor_z
        self.rng = np.random.default_rng(seed + 7919)   # Separate stream: estimator noise is unchanged.
        self.gust = np.zeros(3)

    def ground_ratio(self, height):
        """Cheeseman-Bennett thrust ratio for the rotor plane at this height over the floor."""
        cfg = self.cfg
        gain = cfg.aero_gain * cfg.ground_effect
        if gain <= 0:
            return 1.
        z = max(height, cfg.prop_radius_m / 4 + 1e-3)
        ratio = min(cfg.ground_effect_max, 1. / (1. - (cfg.prop_radius_m / (4. * z)) ** 2))
        return 1. + gain * (ratio - 1.)

    def step(self, position, velocity, body_z, thrust, dt):
        """Return (thrust multiplier, extra acceleration m/s^2) for one step.

        ``thrust`` is the collective specific thrust (m/s^2) before aero.
        """
        cfg = self.cfg
        if cfg.aero_gain <= 0:
            return 1., np.zeros(3)
        rotor_z = position[2] + cfg.rotor_plane_height
        scale = self.ground_ratio(rotor_z - self.floor_z)
        accel = np.zeros(3)
        if cfg.rotor_drag_per_s > 0 and thrust > 0:
            across = velocity - np.dot(velocity, body_z) * body_z
            accel -= cfg.aero_gain * cfg.rotor_drag_per_s * (thrust / GRAVITY) * across
        intensity = 1.
        if self.walls is not None:
            distance, away = self.walls.lookup(np.array([position[0], position[1], rotor_z]))
            gap = distance - cfg.rotor_reach_m
            closeness = min(1., max(0., 1. - gap / cfg.wall_range_m)) if cfg.wall_range_m > 0 else 0.
            if closeness > 0:
                accel -= cfg.aero_gain * cfg.wall_suction * closeness * thrust * away
                intensity += cfg.wall_turbulence * closeness
        if self.world is not None and self.world.in_tunnel(position):
            scale *= 1. + cfg.aero_gain * cfg.tunnel_thrust_change
            intensity += cfg.tunnel_turbulence
        if cfg.turbulence_accel > 0 and cfg.turbulence_time_s > 0:
            tau = cfg.turbulence_time_s
            self.gust += -self.gust * dt / tau + np.sqrt(2. * dt / tau) * self.rng.normal(0., 1., 3)
            accel += cfg.aero_gain * cfg.turbulence_accel * intensity * self.gust
        return scale, accel
