"""The referee counts a touch on the way back from the pad.

Observation (localsim, seed 602, severity 0.7, near ladder, 2026-11-08): the body overlapped the
barrier stand by 9.5 cm during RETURN_DESCEND at (-2.47, 0.59, 0.81), t = 78.2 s
(localsim.clearance), and the report said 0 touches and 'returned to the launch': after the pad the
referee only acted on contacts faster than its crash speed (1 m/s). The team: that run is a fail.

Run from goalnav/: python -m pytest localsim/test_referee_return.py
"""
import json
from pathlib import Path

from localsim.referee import Referee

COURSE = json.loads((Path(__file__).resolve().parents[1] / 'config/course_v3.json').read_text())


def _after_the_pad():
    ref = Referee(COURSE, 0.29)
    ref.left_launch, ref.finished = 0., 60.          # reached the pad at 60 s
    ref.update(61., (-1.6, -2.0, 0.07), (0., 0., 0.), contacts=0, landed=True, armed=True)
    ref.update(62., (-1.6, -2.0, 0.40), (0., 0., 0.3), contacts=0, landed=False, armed=True)   # lifts off
    return ref


def test_a_slow_touch_on_the_return_is_a_touch_and_fails_the_return():
    ref = _after_the_pad()
    ref.update(78.2, (-2.47, 0.59, 0.81), (0.05, 0.02, -0.2), contacts=12, landed=False, armed=True)
    assert len(ref.summary()['touch_events']) == 1
    assert ref.return_failed is not None and 'touch' in ref.return_failed


def test_a_clean_return_still_returns():
    ref = _after_the_pad()
    ref.update(80., (-2.52, 0.86, 0.3), (0., 0., -0.2), contacts=0, landed=False, armed=True)
    ref.update(84., ref.launch, (0., 0., 0.), contacts=0, landed=True, armed=False)
    assert ref.return_done is not None and ref.return_failed is None
    assert ref.summary()['touch_events'] == []
