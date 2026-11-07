"""Conservative finite arithmetic for the reference constant-braking model.

This bounds arithmetic only. Reaction time and minimum braking acceleration
still require independent physical evidence. Unrepresentable intermediate
values raise ValueError; callers must stop rather than treat them as clearance.
"""
import math
from numbers import Real


def _nonnegative(value):
    try:
        if not isinstance(value, Real) or isinstance(value, bool) or value < 0.:
            raise ValueError('Braking inputs must be finite nonnegative reals')
        original = value
        value = float(value)
        if not math.isfinite(value) or (value == 0. and original != 0):
            raise ValueError('Braking inputs must be finite nonnegative reals')
    except (TypeError, OverflowError) as exc:
        raise ValueError('Braking inputs must be finite nonnegative reals') from exc
    return value


def _up(value):
    """Round a nonnegative arithmetic result outward, including underflow."""
    result = math.nextafter(value, math.inf)
    if not math.isfinite(result):
        raise ValueError('Braking arithmetic overflow')
    return result


def stopping_time_factor(speed, reaction_s, deceleration):
    """Upper bound reaction_s + speed/(2*deceleration), in seconds.

    Divide before halving, so a finite deceleration never overflows a doubled
    denominator. Positive operations round outward; tiny nonzero speed is not
    reclassified as rest. The returned factor times speed bounds travel.
    """
    speed, reaction_s, deceleration = map(_nonnegative, (speed, reaction_s, deceleration))
    if deceleration == 0.:
        raise ValueError('Braking deceleration must be positive')
    if speed == 0.:
        return reaction_s
    braking_time = _up(.5 * _up(speed / deceleration))
    return _up(reaction_s + braking_time) if reaction_s else braking_time


def stopping_distance(speed, reaction_s, deceleration):
    """Finite upper bound speed*reaction_s + speed**2/(2*deceleration)."""
    speed = _nonnegative(speed)
    factor = stopping_time_factor(speed, reaction_s, deceleration)
    return _up(speed * factor) if speed else 0.


def speed_for_clearance(available, reaction_s, deceleration, speed_cap):
    """Return a capped speed whose outward-rounded stopping distance fits.

    Rationalizing the quadratic root avoids subtracting nearly equal terms.
    Square roots are divided before squaring could overflow or underflow. A
    bounded final check removes rounding overshoot; no NaN enters a minimum.
    """
    available, reaction_s, deceleration, speed_cap = map(
        _nonnegative, (available, reaction_s, deceleration, speed_cap))
    if deceleration == 0.:
        raise ValueError('Braking deceleration must be positive')
    if available == 0. or speed_cap == 0.:
        return 0.
    if stopping_distance(speed_cap, reaction_s, deceleration) <= available:
        return speed_cap
    root = (math.sqrt(available) / math.sqrt(deceleration)) * math.sqrt(.5)
    half_reaction = .5 * reaction_s
    denominator = half_reaction + math.hypot(half_reaction, root)
    if not math.isfinite(root) or not math.isfinite(denominator) or denominator <= 0.:
        raise ValueError('Braking inverse arithmetic overflow')
    candidate = available / denominator
    if not math.isfinite(candidate):
        raise ValueError('Braking inverse arithmetic overflow')
    candidate = min(speed_cap, candidate)
    for _ in range(8):
        if stopping_distance(candidate, reaction_s, deceleration) <= available:
            return candidate
        candidate = math.nextafter(candidate, 0.)
    raise ValueError('Braking inverse rounding could not establish a safe bound')
