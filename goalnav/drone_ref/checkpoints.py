"""Directed finite-line checkpoint evidence from a measured XY polyline.

Credit means a circular footprint moved from wholly before to wholly after a
declared segment without touching either endpoint, in the required order. It
does not certify registration, obstacle clearance, altitude, landing or event
adjudication. Reference verification blockers remain separate and explicit.
Every straight segment between observations is checked continuously. A caller
must justify that polyline and its uncertainty for physical motion; this module
does not infer a curved inter-sample flight trajectory from two poses.
"""
from dataclasses import asdict, dataclass, field
from itertools import islice
import math
from numbers import Integral, Real


MAX_CHECKPOINTS = 32
MAX_PATH_SAMPLES = 200_000
MAX_COORDINATE_M = 1_000_000.
MAX_FOOTPRINT_RADIUS_M = 1_000.
# A fixed arithmetic allowance over the supported coordinate domain. It is
# independent of sample spacing, so splitting a path does not change policy.
NUMERIC_CLEARANCE_M = 64 * math.ulp(4 * MAX_COORDINATE_M)


def _real(value, label, minimum=None, maximum=None):
    try:
        valid = isinstance(value, Real) and not isinstance(value, bool)
        result = float(value) if valid else math.nan
        valid = valid and math.isfinite(result)
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid or (minimum is not None and result < minimum) or (maximum is not None and result > maximum):
        raise ValueError(label + " is outside its finite supported range")
    return result


def _bounded_items(values, limit, label):
    if isinstance(values, (str, bytes, dict)):
        raise ValueError(label + " must be a bounded sequence")
    try:
        items = tuple(islice(iter(values), limit + 1))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(label + " must be a bounded sequence") from exc
    if len(items) > limit:
        raise ValueError(label + " exceeds the supported count")
    return items


def _xy(value, label="XY point"):
    values = _bounded_items(value, 2, label)
    if len(values) != 2:
        raise ValueError(label + " must contain exactly two values")
    return tuple(_real(v, label, -MAX_COORDINATE_M, MAX_COORDINATE_M) for v in values)


def _identifier(value, label):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 1 <= value <= 2 ** 31 - 1:
        raise ValueError(label + " must be a positive integer")
    return int(value)


@dataclass(frozen=True)
class Checkpoint:
    name: str
    start_xy: tuple
    end_xy: tuple
    forward_normal_xy: tuple
    checkpoint: int | None = None
    route: str | None = None
    length_m: float = field(init=False)
    tangent_xy: tuple = field(init=False, repr=False)

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name or len(self.name) > 128:
            raise ValueError("Checkpoint name must be a nonempty bounded string")
        if self.route is not None and (not isinstance(self.route, str) or not self.route or len(self.route) > 128):
            raise ValueError("Checkpoint route must be a bounded string")
        if self.checkpoint is not None:
            object.__setattr__(self, 'checkpoint', _identifier(self.checkpoint, "Checkpoint ID"))
        start, end = _xy(self.start_xy), _xy(self.end_xy)
        hint = _xy(self.forward_normal_xy, "Forward normal")
        delta = end[0] - start[0], end[1] - start[1]
        length = math.hypot(*delta)
        normal_length = math.hypot(*hint)
        if length <= 2 * NUMERIC_CLEARANCE_M or normal_length == 0:
            raise ValueError("Checkpoint segment or declared normal is degenerate")
        tangent = tuple(v / length for v in delta)
        normalized = tuple(v / normal_length for v in hint)
        if abs(sum(a * b for a, b in zip(tangent, normalized))) > 1e-6:
            raise ValueError("Declared forward direction is not perpendicular to checkpoint")
        perpendicular = -tangent[1], tangent[0]
        sign = 1. if sum(a * b for a, b in zip(perpendicular, normalized)) > 0 else -1.
        # Preserve the explicitly supplied direction. Endpoint document order
        # alone never chooses which side counts as forward.
        object.__setattr__(self, 'start_xy', start)
        object.__setattr__(self, 'end_xy', end)
        object.__setattr__(self, 'tangent_xy', tangent)
        object.__setattr__(self, 'forward_normal_xy', tuple(sign * v for v in perpendicular))
        object.__setattr__(self, 'length_m', length)

    def local(self, xy):
        xy = _xy(xy)
        offset = xy[0] - self.start_xy[0], xy[1] - self.start_xy[1]
        return (sum(a * b for a, b in zip(offset, self.tangent_xy)),
                sum(a * b for a, b in zip(offset, self.forward_normal_xy)))


@dataclass(frozen=True)
class CheckpointReference:
    checkpoints: tuple[Checkpoint, ...]
    required_order: tuple[int, ...]
    chosen_cp4_route: str
    geometry_confirmed: bool
    physical_alignment_verified: bool
    registration_uncertainty_m: float | None
    blockers: tuple[str, ...]


def load_reference(data, route=None):
    """Load supplied CAD-line schema without turning missing evidence into zero.

    The default required order is the complete [1,2,3,4] line task. A different
    task must supply an explicit nonempty unique required_order. CP4 is Gate B or the
    window; ``route`` selects one (default: the reference's chosen_cp4_route).
    A numeric geometry audit may use .checkpoints despite .blockers; deployment
    must separately resolve every blocker and the caller's airframe uncertainty.
    """
    if not isinstance(data, dict) or type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise ValueError("Unsupported checkpoint reference schema")
    route = route or data.get('chosen_cp4_route', 'gate_b')
    if route not in ('gate_b', 'window'):
        raise ValueError("CP4 route must be gate_b or window")
    order = tuple(_identifier(v, "Required checkpoint ID") for v in
                  _bounded_items(data.get('required_order', (1, 2, 3, 4)), MAX_CHECKPOINTS, "Required order"))
    if not order or len(set(order)) != len(order):
        raise ValueError("Required checkpoint order must be nonempty and unique")
    lines = _bounded_items(data.get('checkpoint_lines'), MAX_CHECKPOINTS, "Checkpoint lines")
    selected = {}
    names = set()
    blockers = []
    for line in lines:
        if not isinstance(line, dict):
            raise ValueError("Checkpoint line must be an object")
        number = _identifier(line.get('checkpoint'), "Checkpoint ID")
        name = line.get('name')
        endpoints = _bounded_items(line.get('cad_xy_endpoints_m'), 2, "Checkpoint endpoints")
        if len(endpoints) != 2:
            raise ValueError("Checkpoint needs two finite endpoints")
        checkpoint = Checkpoint(name, endpoints[0], endpoints[1], line.get('forward_normal_cad_xy'),
                                number, name if number == 4 else None)
        if checkpoint.name in names:
            raise ValueError("Duplicate checkpoint name")
        names.add(checkpoint.name)
        if number == 4 and name != route:
            if name not in ('gate_b', 'window'):
                raise ValueError("Unknown CP4 route")
            continue
        if number not in order:
            continue
        if number in selected:
            raise ValueError("Duplicate required checkpoint ID")
        selected[number] = checkpoint
        for flag in ('normal_verified_by_organizer', 'finite_segment_adjudication_verified'):
            value = line.get(flag, False)
            if type(value) is not bool:
                raise ValueError("Checkpoint verification flags must be Boolean")
            if not value:
                blockers.append(checkpoint.name + ': ' + flag + ' is false')
    if set(selected) != set(order):
        raise ValueError("Reference is missing explicitly required checkpoint geometry")
    registration = data.get('svg_to_cad_xy', {})
    if not isinstance(registration, dict):
        raise ValueError("Checkpoint registration metadata must be an object")
    flags = {}
    for name, value in (
            ('geometry_confirmed', data.get('geometry_confirmed', False)),
            ('verification', data.get('verification', False)),
            ('physical_alignment_verified', registration.get('physical_alignment_verified', False))):
        if type(value) is not bool:
            raise ValueError("Reference verification flags must be Boolean")
        flags[name] = value
        if not value:
            blockers.append(name + ' is false')
    uncertainty = registration.get('registration_uncertainty_m')
    if uncertainty is None:
        blockers.append('registration uncertainty is unknown')
    else:
        uncertainty = _real(uncertainty, 'Registration uncertainty', 0., MAX_FOOTPRINT_RADIUS_M)
    return CheckpointReference(tuple(selected[number] for number in order), order, 'gate_b',
                               flags['geometry_confirmed'], flags['physical_alignment_verified'],
                               uncertainty, tuple(blockers))


@dataclass(frozen=True)
class Crossing:
    name: str
    checkpoint: int | None
    segment_index: int
    segment_fraction: float
    path_distance_m: float


@dataclass(frozen=True)
class CheckpointStatus:
    name: str
    checkpoint: int | None
    credited: bool
    phase: str
    approach_observed: bool
    invalid_attempts: int
    reverse_crossings: int
    out_of_order_crossings: int
    last_issue: str | None


@dataclass(frozen=True)
class ProgressReport:
    status: str
    complete: bool
    crossed: tuple[str, ...]
    next_checkpoint: str | None
    samples_seen: int
    footprint_radius_m: float | None
    reason: str
    accepted_crossings: tuple[Crossing, ...]
    checkpoint_statuses: tuple[CheckpointStatus, ...]
    scope: str = 'Declared planar geometry only; no physical registration or challenge qualification'

    def to_dict(self):
        return asdict(self)


@dataclass
class _State:
    credited: bool = False
    phase: str = 'WAIT_APPROACH'
    approach_observed: bool = False
    armed: bool = False
    valid: bool = True
    centre_crossed: bool = False
    last_sign: int = 0
    invalid_attempts: int = 0
    reverse_crossings: int = 0
    out_of_order_crossings: int = 0
    last_issue: str | None = None


def _sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def _endpoint_distance(a, b, endpoint_u):
    dx, dy = b[0] - a[0], b[1] - a[1]
    squared = dx * dx + dy * dy
    weight = ((endpoint_u - a[0]) * dx - a[1] * dy) / squared if squared else 0.
    weight = max(0., min(1., weight))
    return math.hypot(a[0] + weight * dx - endpoint_u, a[1] + weight * dy)


class CheckpointMonitor:
    """Bounded streaming whole-footprint evidence, identical to report_path.

    update(xy) returns a frozen ProgressReport. Malformed observations or count
    overflow latch UNKNOWN; no later point can silently repair missing evidence.
    A failed geometric attempt can be retried from a fully observed approach
    side. Already credited checkpoints are never credited a second time.
    The circular sweep avoids both closed segment endpoints, and the centre
    must cross the finite interior. Together these establish full-footprint
    passage rather than merely intersecting the infinite supporting line.
    A fixed arithmetic allowance (about 30 nm over supported coordinates) and
    the same tiny ordering-confirmation distance are conservative numerical
    checks, never substitutes for measured registration/airframe uncertainty.
    """

    def __init__(self, checkpoints, footprint_radius_m, max_samples=MAX_PATH_SAMPLES):
        self.checkpoints = _bounded_items(checkpoints, MAX_CHECKPOINTS, "Checkpoints")
        if not self.checkpoints or any(not isinstance(cp, Checkpoint) for cp in self.checkpoints):
            raise ValueError("Need a nonempty sequence of validated Checkpoint objects")
        if len({cp.name for cp in self.checkpoints}) != len(self.checkpoints):
            raise ValueError("Checkpoint names must be unique")
        self.radius = _real(footprint_radius_m, "Footprint radius", 0., MAX_FOOTPRINT_RADIUS_M)
        if self.radius <= 0:
            raise ValueError("Whole-footprint crossing requires a positive radius")
        if isinstance(max_samples, bool) or not isinstance(max_samples, Integral) or not 1 <= max_samples <= MAX_PATH_SAMPLES:
            raise ValueError("Path sample cap must be a bounded positive integer")
        self.max_samples = int(max_samples)
        self.states = [_State() for _ in self.checkpoints]
        self.samples_seen = 0
        self.next_index = 0
        self.accepted = []
        self.pending = []
        self.previous = None
        self.path_distance = 0.
        self.fault = None
        for cp, state in zip(self.checkpoints, self.states):
            if cp.length_m <= 2 * (self.radius + NUMERIC_CLEARANCE_M):
                state.last_issue = 'finite checkpoint is too narrow for the footprint'

    def _invalid_attempt(self, state, reason):
        if state.valid:
            state.invalid_attempts += 1
        state.valid = False
        state.last_issue = reason

    def _advance(self, checkpoint, state, start, end):
        if state.credited:
            return None
        a, b = checkpoint.local(start), checkpoint.local(end)
        full = self.radius + NUMERIC_CLEARANCE_M
        if a[1] < -full:
            state.armed, state.valid, state.centre_crossed = True, True, False
            state.approach_observed = True
        end_sign = _sign(b[1])
        intersection_u = None
        if state.last_sign and end_sign and state.last_sign != end_sign:
            weight = -a[1] / (b[1] - a[1]) if b[1] != a[1] else 0.
            weight = max(0., min(1., weight))
            intersection_u = a[0] + weight * (b[0] - a[0])
        if (state.last_sign == 1 and end_sign == -1 and intersection_u is not None
                and NUMERIC_CLEARANCE_M < intersection_u < checkpoint.length_m - NUMERIC_CLEARANCE_M):
            state.reverse_crossings += 1
            state.last_issue = 'reverse-direction centreline crossing does not earn credit'
        crossing = None
        if state.armed:
            if min(_endpoint_distance(a, b, 0.), _endpoint_distance(a, b, checkpoint.length_m)) <= full:
                self._invalid_attempt(state, 'footprint touches or passes a finite checkpoint endpoint')
            if state.last_sign == -1 and end_sign == 1:
                if not NUMERIC_CLEARANCE_M < intersection_u < checkpoint.length_m - NUMERIC_CLEARANCE_M:
                    self._invalid_attempt(state, 'centreline bypasses the finite checkpoint segment')
                else:
                    state.centre_crossed = True
            if b[1] > full:
                if state.valid and state.centre_crossed:
                    crossing = max(0., min(1., (full - a[1]) / (b[1] - a[1])))
                elif not state.last_issue:
                    state.last_issue = 'no valid directed finite-segment crossing'
                state.armed = False
                state.phase = 'EXIT_PENDING' if crossing is not None else 'WAIT_APPROACH'
            else:
                state.phase = 'TRANSIT' if state.valid else 'INVALID_TRANSIT'
        elif b[1] > full and not state.approach_observed:
            state.last_issue = 'no fully observed approach-side footprint'
        if b[1] < -full:
            state.armed, state.valid, state.centre_crossed = True, True, False
            state.approach_observed = True
            state.phase = 'APPROACH'
        if end_sign:
            state.last_sign = end_sign
        return crossing

    def update(self, xy):
        if self.fault is not None:
            return self.snapshot()
        if self.samples_seen >= self.max_samples:
            self.fault = 'path sample budget exceeded; evidence incomplete'
            return self.snapshot()
        try:
            point = _xy(xy)
        except (TypeError, ValueError, OverflowError) as exc:
            self.fault = str(exc)
            return self.snapshot()
        self.samples_seen += 1
        if self.previous is None:
            full = self.radius + NUMERIC_CLEARANCE_M
            for cp, state in zip(self.checkpoints, self.states):
                signed = cp.local(point)[1]
                state.last_sign = _sign(signed)
                if signed < -full:
                    state.armed = state.approach_observed = True
                    state.phase = 'APPROACH'
            self.previous = point
            return self.snapshot()
        length = math.dist(self.previous, point)
        candidates = []
        for index, (cp, state) in enumerate(zip(self.checkpoints, self.states)):
            fraction = self._advance(cp, state, self.previous, point)
            if fraction is not None:
                candidates.append((fraction, index))
        for fraction, index in candidates:
            cp = self.checkpoints[index]
            event = Crossing(cp.name, cp.checkpoint, self.samples_seen - 2,
                             fraction, self.path_distance + fraction * length)
            self.pending.append((event.path_distance_m, index, event))
        self.pending.sort(key=lambda value: value[:2])
        if len(self.pending) > MAX_CHECKPOINTS:
            self.fault = 'pending crossing budget exceeded'
        for before, after in zip(self.pending, self.pending[1:]):
            if after[0] - before[0] <= NUMERIC_CLEARANCE_M:
                self.fault = 'simultaneous checkpoint completions cannot establish strict order'
                break
        if self.fault is None:
            retained = []
            for distance, index, event in self.pending:
                # A tiny fixed confirmation distance prevents a subdivision
                # between numerically simultaneous completions from creating
                # an order that a coarser version of the same path cannot prove.
                if self.path_distance + length - distance <= NUMERIC_CLEARANCE_M:
                    retained.append((distance, index, event))
                    continue
                cp, state = self.checkpoints[index], self.states[index]
                if index != self.next_index:
                    state.out_of_order_crossings += 1
                    state.last_issue = 'crossed before the required preceding checkpoint'
                    state.phase = 'WAIT_APPROACH'
                    continue
                state.credited = True
                state.phase = 'CROSSED'
                state.last_issue = None
                self.accepted.append(event)
                self.next_index += 1
            self.pending = retained
        self.previous = point
        self.path_distance += length
        return self.snapshot()

    def invalidate_evidence(self, reason):
        """Latch UNKNOWN without erasing historical observations or crossings."""
        if not isinstance(reason, str) or not reason.strip() or len(reason) > 512:
            raise ValueError('Evidence invalidation requires a bounded nonempty reason')
        if self.fault is None:
            self.fault = reason
        return self.snapshot()

    def snapshot(self):
        complete = self.fault is None and self.next_index == len(self.checkpoints)
        status = 'UNKNOWN' if self.fault is not None or self.samples_seen == 0 else 'COMPLETE' if complete else 'INCOMPLETE'
        next_name = None if complete else self.checkpoints[self.next_index].name if self.next_index < len(self.checkpoints) else None
        if self.fault:
            reason = self.fault
        elif complete:
            reason = 'all declared whole-footprint checkpoints crossed in order'
        elif not self.samples_seen:
            reason = 'no path observations'
        else:
            state = self.states[self.next_index]
            reason = state.last_issue or ('awaiting numeric ordering confirmation' if state.phase == 'EXIT_PENDING'
                                         else 'awaiting whole-footprint exit' if state.armed
                                         else 'awaiting approach-side evidence')
        statuses = tuple(CheckpointStatus(cp.name, cp.checkpoint, state.credited, state.phase,
                                          state.approach_observed, state.invalid_attempts,
                                          state.reverse_crossings, state.out_of_order_crossings, state.last_issue)
                         for cp, state in zip(self.checkpoints, self.states))
        return ProgressReport(status, complete, tuple(c.name for c in self.accepted), next_name,
                              self.samples_seen, self.radius, reason, tuple(self.accepted), statuses)


def report_path(checkpoints, path_xy, footprint_radius_m, max_samples=MAX_PATH_SAMPLES):
    """Offline replay using exactly the streaming monitor's geometry semantics."""
    try:
        monitor = CheckpointMonitor(checkpoints, footprint_radius_m, max_samples)
    except (TypeError, ValueError, OverflowError) as exc:
        return ProgressReport('UNKNOWN', False, (), None, 0, None, str(exc), (), ())
    try:
        if isinstance(path_xy, (str, bytes, dict)):
            raise ValueError('Path must be an iterable of XY observations')
        for point in path_xy:
            report = monitor.update(point)
            if report.status == 'UNKNOWN':
                break
    except (TypeError, ValueError, OverflowError) as exc:
        monitor.fault = 'invalid path input: ' + str(exc)
    return monitor.snapshot()
