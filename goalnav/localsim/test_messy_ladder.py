"""The stepladder's named poses and the switch between them (localsim.messy_ladder).

    cd goalnav && python -m pytest localsim/test_messy_ladder.py
"""
import copy
import math

import numpy as np
import pytest

from localsim import goalrun, messy_course as mc, messy_ladder as ml


def test_both_poses_kept_and_original_is_the_default():
    # The team switched back to the original spot on 2026-11-08 (the move may have been temporary).
    assert set(ml.POSES) >= {'original', 'moved'}
    assert ml.DEFAULT_POSE == 'original'
    original = ml.POSES['original']
    assert original.centre == (-2.31, -0.62) and original.yaw_deg == 16.     # as first measured, unchanged


def test_moved_pose_is_in_the_south_west_corner():
    moved = ml.POSES['moved']
    assert moved.centre[0] < -2.6 and moved.centre[1] < -2.3                  # far from the original
    assert math.dist(moved.centre, ml.POSES['original'].centre) > 1.5


def test_mesh_defaults_follow_the_default_pose():
    v = np.vstack([g[0] for g in ml.ladder().values()])
    centre = np.asarray(ml.POSES[ml.DEFAULT_POSE].centre)
    assert np.linalg.norm(v[:, :2].mean(axis=0) - centre) < 0.15


def test_goalrun_flag_parses():
    parser = goalrun.build_parser()
    assert parser.parse_args([]).ladder_pose == ml.DEFAULT_POSE
    assert parser.parse_args(['--ladder-pose', 'original']).ladder_pose == 'original'
    with pytest.raises(SystemExit):
        parser.parse_args(['--ladder-pose', 'nowhere'])


@pytest.mark.parametrize('pose', ['original', 'moved'])
@pytest.mark.parametrize('seed,severity', [(0, 0.0), (1, 0.7), (2, 1.0)])
def test_courses_generate_and_validate_with_each_pose(pose, seed, severity):
    course = mc.generate(seed, severity, ladder_pose=pose)
    # generate() adds the next course's frame copies after validating (they stand outside this net).
    own = copy.copy(course)
    own.groups = {k: g for k, g in course.groups.items() if not k.endswith('(next course)')}
    assert not mc.validate(own)
    assert course.ladder['pose'] == pose and course.ladder['clear']
    want = np.asarray(ml.POSES[pose].centre)
    assert np.linalg.norm(np.asarray(course.ladder['centre']) - want) <= 0.25   # jitter (0.13 at severity 1) + slide (0.2)
    v = np.vstack([g[0] for k, g in course.groups.items() if k.startswith('Ladder')])
    nets = course.nets                                                        # inside the net it stands in
    assert v[:, 0].min() >= -nets['west'] and v[:, 1].min() >= -nets['south']
    assert v[:, 0].min() >= -(mc.NET + ml.NET_TOLERANCE) and v[:, 1].min() >= -(mc.NET + ml.NET_TOLERANCE)
