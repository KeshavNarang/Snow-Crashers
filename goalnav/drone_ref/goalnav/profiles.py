"""Risk profiles: one file of named flight-parameter sets (``config/goalnav_profiles.yaml``).

Each key of a profile names a field of the mission, planner or follower
config (``MissionConfig``, ``PlanConfig``, ``FollowConfig``); ``overrides``
(a dict, e.g. from a ROS parameter) is applied on top. Unknown keys are an
error, so a typo never silently flies the default.
"""
from dataclasses import fields
from pathlib import Path

import yaml

from drone_ref.goalnav.control import FollowConfig
from drone_ref.goalnav.mission import MissionConfig
from drone_ref.goalnav.planner import PlanConfig

from drone_ref.goalnav import resource

DEFAULT_FILE = resource('config/goalnav_profiles.yaml')


def load(name='safe', path=DEFAULT_FILE, overrides=None):
    """(MissionConfig, PlanConfig, FollowConfig) for a named profile."""
    profiles = yaml.safe_load(Path(path).read_text())
    if name not in profiles:
        raise ValueError(f'unknown profile {name!r}; have {sorted(profiles)}')
    values = dict(profiles[name] or {})
    values.update(overrides or {})
    built = []
    for kind in (MissionConfig, PlanConfig, FollowConfig):
        names = {f.name for f in fields(kind)}
        chosen = {k: values.pop(k) for k in list(values) if k in names}
        built.append(kind(**chosen))
    if values:
        raise ValueError(f'profile {name!r}: unknown settings {sorted(values)}')
    return tuple(built)
