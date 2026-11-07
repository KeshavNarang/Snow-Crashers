"""Path following: pure pursuit on the planned path, speed and yaw limits.

PX4 does the velocity and attitude control. This turns the latest path into
a velocity, heading and climb-rate setpoint at the control rate.
"""
from dataclasses import dataclass
import math

import numpy as np

from drone_ref.goalnav.geometry import wrap


@dataclass
class FollowConfig:
    lookahead: float = 0.30       # m at rest
    lookahead_per_speed: float = 0.40
    cross_track_gain: float = 2.0  # 1/s toward the nearest path point
    corner_guard: bool = True      # apply it: pure pursuit alone cuts inside corners
    track_accel_gain: float = 4.0  # acceleration feedforward per metre off the path (full setpoints)
    track_accel_damping: float = 3.0
    track_max_accel: float = 2.0
    max_lookahead: float = 1.1
    max_accel: float = 2.5        # m/s^2 slew on the commanded velocity
    yaw_rate: float = 1.4         # rad/s
    yaw_lookahead: float = 1.0
    z_gain: float = 1.5
    max_climb: float = 1.0
    max_sink: float = 0.6


def _closest(path, p):
    """(segment index, fraction) of the path point nearest p."""
    best, where = math.inf, (0, 0.)
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        d = b - a
        t = float(np.clip((p - a) @ d / max(d @ d, 1e-9), 0., 1.))
        dist = float(np.linalg.norm(a + t * d - p))
        if dist < best:
            best, where = dist, (i, t)
    return where


def point_ahead(path, p, distance):
    """The path point ``distance`` metres ahead of the one nearest p."""
    i, t = _closest(path, p)
    here = path[i] + t * (path[i + 1] - path[i])
    remaining = distance
    for j in range(i, len(path) - 1):
        start = here if j == i else path[j]
        seg = float(np.linalg.norm(path[j + 1] - start))
        if seg >= remaining:
            return start + (path[j + 1] - start) * (remaining / max(seg, 1e-9))
        remaining -= seg
    return path[-1]


def heading_change(path, p, distance=1.2):
    """Total turn (radians) along the next ``distance`` metres of path."""
    points = [point_ahead(path, p, d) for d in np.linspace(0., distance, 7)]
    angles = [math.atan2(*(b - a)[::-1]) for a, b in zip(points, points[1:]) if np.linalg.norm(b - a) > 1e-3]
    return sum(abs(wrap(b - a)) for a, b in zip(angles, angles[1:]))


class Follower:
    def __init__(self, config=None):
        self.cfg = config or FollowConfig()
        self.velocity = np.zeros(2)
        self.yaw = None

    def reset(self, yaw):
        self.velocity = np.zeros(2)
        self.yaw = yaw

    def step(self, dt, path, position_xy, speed, yaw_now):
        """Horizontal velocity and yaw setpoints toward the path."""
        cfg = self.cfg
        p = np.asarray(position_xy, float)
        if path is None or len(path) < 2:
            wanted = np.zeros(2)
            target_yaw = self.yaw if self.yaw is not None else yaw_now
        else:
            path = [np.asarray(q, float) for q in path]
            look = min(cfg.max_lookahead, cfg.lookahead + cfg.lookahead_per_speed * float(np.linalg.norm(self.velocity)))
            carrot = point_ahead(path, p, look)
            offset = carrot - p
            distance = float(np.linalg.norm(offset))
            end = float(np.linalg.norm(path[-1] - p))
            v = min(speed, 1.5 * end + 0.05)          # Settle onto the path's end.
            wanted = offset / distance * v if distance > 1e-3 else np.zeros(2)
            i, t = _closest(path, p)
            nearest = path[i] + t * (path[i + 1] - path[i])
            self.nearest = nearest
            seg = path[i + 1] - path[i]
            self.tangent = seg / max(float(np.linalg.norm(seg)), 1e-9)
            if cfg.corner_guard:
                wanted = wanted + cfg.cross_track_gain * (nearest - p)
            ahead = point_ahead(path, p, cfg.yaw_lookahead) - p
            target_yaw = math.atan2(ahead[1], ahead[0]) if np.linalg.norm(ahead) > 0.15 else self.yaw
        change = wanted - self.velocity
        limit = cfg.max_accel * dt
        if np.linalg.norm(change) > limit:
            change *= limit / np.linalg.norm(change)
        self.velocity = self.velocity + change
        if self.yaw is None:
            self.yaw = yaw_now
        if target_yaw is not None:
            step = float(np.clip(wrap(target_yaw - self.yaw), -cfg.yaw_rate * dt, cfg.yaw_rate * dt))
            self.yaw = wrap(self.yaw + step)
        return self.velocity.copy(), self.yaw

    def track_accel(self, position_xy, velocity_xy):
        """Acceleration feedforward pulling onto the path's line (for a full
        PX4 setpoint): straight to thrust, without the velocity loop's lag."""
        cfg = self.cfg
        if getattr(self, 'nearest', None) is None:
            return np.zeros(2)
        normal = np.array([-self.tangent[1], self.tangent[0]])
        off = float((self.nearest - np.asarray(position_xy, float)) @ normal)
        v_off = float(np.asarray(velocity_xy, float) @ normal)
        a = normal * (cfg.track_accel_gain * off - cfg.track_accel_damping * v_off)
        norm = float(np.linalg.norm(a))
        return a * (cfg.track_max_accel / norm) if norm > cfg.track_max_accel else a

    def climb(self, height, target):
        cfg = self.cfg
        return float(np.clip(cfg.z_gain * (target - height), -cfg.max_sink, cfg.max_climb))
