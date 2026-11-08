"""The return home: from the pad, over the barrier, down onto the launch mark.

Everything the return's safety depends on, in one place and free of the
mission's state, so a planner can take it whole:

- **Margins** from one clearance target: a quarter of the tunnel's radius
  (0.184 m) from the rotor tips. Sideways, the height profile looks
  ``reach`` = rotor radius + target from the line (the mission's old 0.45 m
  left 0.16 m); upward, the feet clear the tallest thing near the line by
  ``vertical_margin`` (old 0.15 m). The touchdown spot keeps ``reach`` from
  each wall it is measured against (the barrier's configured 0.42 m left
  0.13 m) where the corridor has room.
- **Spot freeze:** the touchdown spot follows the walls measured on the way
  in (``return_spot.ReturnSpot``, which corrects the odometry's drift) only
  while the body is above the wall tops. Below them a moving target means a
  sideways move next to a wall, the one place the return scraped in the
  simulator (2026-11-07 batches).
- **Descent gate:** beside the walls, come down only while on the spot
  (``hold_off`` / ``resume_off``, with hysteresis) and not sliding sideways;
  otherwise hold the height and re-centre. Never for more than ``hold_max_s``
  in all (then on down, slowly: a hover until the battery goes is worse), and
  never once the feet are nearly down.

Heights are of the body origin over the floor (the feet are
``feet_below_origin`` under it), as ``mission`` keeps them.
"""
from dataclasses import dataclass

TUNNEL_RADIUS = 0.7366          # the tunnel's half width (localsim.messy_course.TUBE['hw'])


@dataclass
class HomewardConfig:
    body_radius: float = 0.29             # rotor tips from the centre (Sky Beta CAD, 7 in props)
    clear_target: float = TUNNEL_RADIUS / 4
    vertical_margin: float = 0.19         # feet over the tallest thing near the line (>= clear_target)
    feet_below_origin: float = 0.07       # mission.landed_base_height
    max_height: float = 2.2               # origin; the net roof is at ~3 m
    wall_top: float = 1.73                # panels 1.73 m (localsim WALL_H); the barrier's top
    freeze_margin: float = 0.05
    fast_rate: float = 0.6                # m/s down above slow_below (mission's old values)
    slow_rate: float = 0.3
    slow_below: float = 0.6
    hold_off: float = 0.08                # beside the walls: hold when this far off the spot...
    resume_off: float = 0.04              # ...and go on once back within this
    hold_speed: float = 0.15              # or when sliding sideways faster than this (m/s)
    resume_speed: float = 0.08
    hold_max_s: float = 4.0               # in all, over the whole descent
    ground_h: float = 0.15                # feet nearly down: never hold

    @property
    def reach(self):
        """Centre-to-obstacle distance the return keeps sideways."""
        return self.body_radius + self.clear_target

    @property
    def freeze_below(self):
        """Origin height under which the body is beside the walls."""
        return self.wall_top + self.feet_below_origin + self.freeze_margin


def face_offset(offset, cfg):
    """A touchdown spot's distance from a measured wall face: as configured, but
    never under ``reach``."""
    return max(float(offset), cfg.reach)


def cruise_height(top, cfg):
    """Origin height that clears an obstacle ``top`` m tall, capped at ``max_height``."""
    return min(float(top) + cfg.vertical_margin + cfg.feet_below_origin, cfg.max_height)


def spot_may_move(h, cfg):
    """True while the touchdown spot may still follow new wall measurements."""
    return h >= cfg.freeze_below


class DescentGate:
    """Descent rate (m/s, positive down) for the straight-down part of the return."""

    def __init__(self, cfg=None):
        self.cfg = cfg or HomewardConfig()
        self.holding = False
        self.held_s = 0.
        self.holds = 0              # times it stopped to re-centre (for the log)

    def rate(self, h, off, speed, dt=0.):
        """``h`` origin height over the floor, ``off`` distance from the spot (m),
        ``speed`` horizontal speed (m/s), ``dt`` since the last call (s)."""
        cfg = self.cfg
        going = cfg.fast_rate if h > cfg.slow_below else cfg.slow_rate
        if h < cfg.ground_h:
            self.holding = False
            return cfg.slow_rate
        if h >= cfg.freeze_below:
            self.holding = False
            return going
        if self.held_s >= cfg.hold_max_s:
            self.holding = False
            return cfg.slow_rate
        if self.holding:
            if off <= cfg.resume_off and speed <= cfg.resume_speed:
                self.holding = False
        elif off > cfg.hold_off or speed > cfg.hold_speed:
            self.holding = True
            self.holds += 1
        if self.holding:
            self.held_s += dt
            return 0.
        return going
