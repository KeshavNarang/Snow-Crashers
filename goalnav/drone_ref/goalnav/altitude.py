"""Height above the floor: the inertial estimate first, the rangefinder second.

PX4's local z (visual-inertial odometry fused with the IMU) is smooth and
does not jump when something passes under the drone. The downward VL53L1X
reads the nearest surface in its cone, so a boulder, a tarp skirt or the
tunnel cradle makes the floor look up to half a metre higher. So:

- height above the floor = local z minus the floor's local z;
- a range reading corrects the floor's local z (slowly, removing VIO
  vertical drift) only when it agrees with the prediction within ``gate``;
- a reading clearly shorter than predicted is an obstacle under the drone,
  reported so the live map can mark it, never a change of height;
- the last ``settle`` readings longer than predicted (obstacles between them
  aside), if they agree with each other, are the floor after VIO has drifted past the gate: their median
  corrects it too. (Something under the drone only ever reads shorter than
  the floor.) Without this, a drift past ``gate`` locks the rangefinder out
  for the rest of the run.
"""
from dataclasses import dataclass


@dataclass
class HeightConfig:
    gate: float = 0.04          # m; a range further from the prediction is not the floor
    gain: float = 0.02          # per accepted reading
    max_correction: float = 0.10  # VIO vertical drift over one run is centimetres
    min_range: float = 0.12     # the VL53L1X is unreliable this close
    max_range: float = 3.5
    max_tilt_cos: float = 0.94  # cos(20 deg): above this tilt the cone hits the floor obliquely
    settle: int = 25            # agreeing long readings (0.5 s at 50 Hz) that re-lock the floor
    agree: float = 0.05         # m; their spread (the VL53L1X's noise is ~+-2 cm)


class FloorTracker:
    def __init__(self, floor_z, config=None):
        self.cfg = config or HeightConfig()
        self.floor_z = self.initial = float(floor_z)
        self.accepted = 0
        self.rejected = 0
        self.last = None            # ('floor' | 'obstacle' | 'far', range)
        self.long = []              # recent errors of readings longer than the gate

    def height(self, z):
        """Height of the body origin above the floor."""
        return z - self.floor_z

    def update(self, z, distance, tilt_cos, floor_clear=True):
        """Feed one range sample (body origin to the nearest surface below,
        vertical component). Returns ('obstacle', top_z) for a surface well
        above the floor, else None."""
        cfg = self.cfg
        if distance is None or tilt_cos < cfg.max_tilt_cos or not cfg.min_range <= distance <= cfg.max_range:
            self.last = ('far', distance)
            return None
        predicted = z - self.floor_z
        error = distance - predicted
        if error > cfg.gate:
            self.long = self.long[-(cfg.settle - 1):] + [error]
        elif error >= -cfg.gate:
            self.long = []                  # Back on the floor (short readings are obstacles: kept)
        relock = len(self.long) == cfg.settle and max(self.long) - min(self.long) <= cfg.agree
        if relock:
            error = sorted(self.long)[cfg.settle // 2]
        if (abs(error) <= cfg.gate and floor_clear) or relock:
            self.floor_z = min(max(self.floor_z - cfg.gain * error, self.initial - cfg.max_correction),
                               self.initial + cfg.max_correction)
            self.accepted += 1
            self.last = ('floor', distance)
            return None
        self.rejected += 1
        if error < -2 * cfg.gate:
            self.last = ('obstacle', distance)
            return ('obstacle', z - distance)
        self.last = ('far', distance)       # Longer than the floor: a hole or a bad return.
        return None
