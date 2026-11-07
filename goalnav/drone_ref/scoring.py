"""Offline rulebook arithmetic; not the organizers' scoring authority."""
from dataclasses import dataclass
import math

CHECKPOINT_POINTS = (4., 7., 10., 7., 7.)


@dataclass(frozen=True)
class Score:
    checkpoint: float
    speed: float
    window: float
    return_bonus: float
    run: float
    total: float


def score_run(checkpoints, elapsed, fastest_time, *, touches=0,
              over_wall=(), window=False, returned=False,
              technical=0., generality=0.):
    """Score against the fastest *eligible* time to the same last checkpoint.

    elapsed is raw time to final reached checkpoint/finish; touches counts only
    touches before that timestamp. For over-wall times organizers select the
    fastest legal reference, not this run. Pass over_wall as 1-based checkpoint
    numbers that were reached using overflight since the preceding checkpoint.
    """
    if isinstance(checkpoints, bool) or not isinstance(checkpoints, int) or not 0 <= checkpoints <= 5:
        raise ValueError("checkpoints must be integer 0..5")
    if isinstance(touches, bool) or not isinstance(touches, int) or touches < 0:
        raise ValueError("touches must be a nonnegative integer")
    try:
        numeric = all(not isinstance(x, bool) and math.isfinite(x)
                      for x in (elapsed, fastest_time, technical, generality))
    except (TypeError, OverflowError):
        numeric = False
    if not numeric:
        raise ValueError("Nonfinite score input")
    elapsed, fastest_time, technical, generality = map(
        float, (elapsed, fastest_time, technical, generality))
    if not isinstance(window, bool) or not isinstance(returned, bool):
        raise ValueError("Route bonus flags must be booleans")
    if elapsed < 0 or fastest_time < 0:
        raise ValueError("Times cannot be negative")
    if not 0 <= technical <= 25 or not 0 <= generality <= 15:
        raise ValueError("Judged points out of bounds")
    try:
        walls = list(over_wall)
    except TypeError as exc:
        raise ValueError("Over-wall checkpoints must be an iterable of integers") from exc
    if any(isinstance(c, bool) or not isinstance(c, int) or not 1 <= c <= checkpoints for c in walls):
        raise ValueError("Over-wall checkpoint not reached")
    walls = set(walls)
    if checkpoints == 0:
        if returned or window:
            raise ValueError("Route bonus requires a finish")
        return Score(0., 0., 0., 0., 0., technical + generality)
    try:
        adjusted = float(elapsed) + 5.0 * touches
    except OverflowError as exc:
        raise ValueError("Penalty-adjusted time overflow") from exc
    if elapsed <= 0 or fastest_time <= 0 or adjusted <= 0 or not math.isfinite(adjusted):
        raise ValueError("Reached-checkpoint times must be positive")
    if not walls and fastest_time > adjusted * (1. + 1e-9):
        raise ValueError("Fastest eligible time cannot exceed this eligible run's time")
    # An illegal over-wall run may be faster than all legal runs. Cap speed
    # credit so no run exceeds its stated 7 or 20 point allowance.
    ratio = min(1., fastest_time / adjusted)
    checkpoint = sum(p * (.15 if i + 1 in walls else 1.)
                     for i, p in enumerate(CHECKPOINT_POINTS[:checkpoints]))
    finished = checkpoints == 5
    if not finished and (window or returned):
        raise ValueError("Bonuses require a finish")
    speed = (20. if finished else 7.) * ratio
    route = 5. if window else 0.
    base = checkpoint + speed + route
    bonus = base * .25 if returned else 0.
    return Score(checkpoint, speed, route, bonus, base + bonus, base + bonus + technical + generality)
