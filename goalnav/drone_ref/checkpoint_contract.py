"""Event-specific checkpoint contract checks shared by every eligibility path.

The geometry monitor deliberately supports smaller ordered tasks. Event claims
must instead bind all four stages to the declared course source. Matching hashes
are consistency assertions, not authenticated provenance or physical review.
"""
from dataclasses import replace

from .checkpoints import load_reference


EVENT_REQUIRED_ORDER = (1, 2, 3, 4)
_EMBEDDED_REFERENCE = object()
_HEX_DIGITS = frozenset('0123456789abcdefABCDEF')


def _source_digest(container, *keys):
    """Return a normalized SHA-256 identifier, or None for malformed metadata."""
    value = container
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    if (not isinstance(value, str) or len(value) != 64
            or any(character not in _HEX_DIGITS for character in value)):
        return None
    return value.lower()


def load_event_reference(course, reference_data=_EMBEDDED_REFERENCE, route=None):
    """Load geometry and add event order/source blockers without a bypass flag.

    Return the same CheckpointReference type as load_reference. Malformed
    geometry raises ValueError. Missing, invalid or mismatched source identifiers
    and non-event task declarations remain explicit blockers, so callers can
    still display a geometry-only numerical report without claiming eligibility.

    course.source.sha256 names the course OBJ source; sources.course_obj.sha256
    in the reference must name the same source. Historical reviewed-course JSON
    hashes are intentionally not compared with subsequently rebuilt artifacts.
    """
    if not isinstance(course, dict):
        raise ValueError('Event course must be an object')
    data = (course.get('checkpoint_reference')
            if reference_data is _EMBEDDED_REFERENCE else reference_data)
    reference = load_reference(data, route)
    blockers = list(reference.blockers)
    if 'required_order' not in data or reference.required_order != EVENT_REQUIRED_ORDER:
        blockers.append('event required_order must explicitly be [1, 2, 3, 4]')
    if data.get('chosen_cp4_route') != 'gate_b':
        blockers.append('event chosen_cp4_route must explicitly be gate_b')

    course_digest = _source_digest(course, 'source', 'sha256')
    checkpoint_digest = _source_digest(data, 'sources', 'course_obj', 'sha256')
    if course_digest is None:
        blockers.append('course source.sha256 must be a present valid SHA-256 identifier')
    if checkpoint_digest is None:
        blockers.append('checkpoint sources.course_obj.sha256 must be a present valid SHA-256 identifier')
    if course_digest is not None and checkpoint_digest is not None and course_digest != checkpoint_digest:
        blockers.append('checkpoint reference belongs to a different course OBJ')
    return replace(reference, blockers=tuple(blockers))
