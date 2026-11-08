"""Evals for the course's wall layouts (localsim.messy_course).

'scan' is the course as scanned on 2026-11-06; 'rebuilt' is the course as
rebuilt on 2026-11-07, measured from photos/Changed_Walls.jpeg and
photos/New_Walls_*. Measurements and their uncertainty:
agents/logs/2026-11-07-walls-rebuilt.md.

Run from goalnav/: python -m pytest localsim/test_messy_course_layout.py
"""
import math

import numpy as np
import pytest

from localsim import goalrun
from localsim import messy_course as mc

CASES = [(0, 0.0), (1, 1.0)]     # the measured pose, and the full jitter (CLAUDE.md budgets)


def _by_name(walls):
    return {w.name: w for w in walls}


def _from_north(wall):
    """Degrees from north of the wall's base -> free end, + toward east."""
    a, b = (np.asarray(p) for p in (wall.pieces[0][0], wall.pieces[-1][1]))
    d = b - a
    return math.degrees(math.atan2(d[0], d[1]))


def _length(wall):
    return sum(float(np.linalg.norm(np.asarray(b) - np.asarray(a))) for a, b in wall.pieces)


@pytest.fixture(scope='module')
def rebuilt():
    return {(s, v): mc.generate(s, v) for s, v in CASES}


def test_layout_switch_defaults_to_the_rebuild():
    assert set(mc.LAYOUTS) == {'scan', 'rebuilt'}
    assert mc.LAYOUT == 'rebuilt'


def test_goalrun_offers_the_layout_switch(capsys):
    with pytest.raises(SystemExit):
        goalrun.main(['--help'])
    assert '--layout' in capsys.readouterr().out


def test_scan_layout_is_the_course_as_scanned():
    """Switching back gives the old walls: M2 there, M1 turned ~6 degrees, M3 north-south."""
    for seed in (0,):
        c = mc.generate(seed, 0.0, layout='scan')
        names = {w.name for w in c.walls if not w.name.startswith('Barrier')}
        assert names == {'M1', 'M2', 'M3', 'C1', 'A1', 'C2a'}
        by = _by_name(c.walls)
        assert abs(_from_north(by['M1']) - 6.) < 0.5          # SCAN_YAW -6 (CCW), north end east
        assert abs(_from_north(by['M3'])) < 0.5
        assert 0.9 < by['M3'].pieces[0][0][0] < 1.1            # the scan's M3, 0.70 east of the CAD


def test_rebuild_tilts_m1_30_to_60_degrees_east(rebuilt):
    for (seed, sev), c in rebuilt.items():
        angle = _from_north(_by_name(c.walls)['M1'])
        assert 30. - 1e-6 <= angle <= 60. + 1e-6, (seed, sev, angle)
        if sev == 0:
            assert abs(angle - 45.) < 1e-6


def test_rebuild_moves_m3_to_the_photos_pose(rebuilt):
    """A short panel standing free just north of the barrier, centred on its two
    measured pavers' stand, its north end 35.5 degrees east of north; its
    stand's feet and pavers on its north-west side."""
    for (seed, sev), c in rebuilt.items():
        m3 = _by_name(c.walls)['M3']
        a, b = (np.asarray(p) for p in m3.pieces[0])
        reach = 0.10 * mc.JITTER * sev + 1e-6
        assert np.all(np.abs((a + b) / 2 - np.asarray(mc.M3_CENTRE)) <= reach), (seed, sev, (a + b) / 2)
        angle = _from_north(m3)
        assert abs(angle - mc.M3_FROM_NORTH_DEG) <= 4. * mc.JITTER * sev + 1e-6, (seed, sev, angle)
        assert abs(_length(m3) - mc.SHORT_PANEL) < 0.06, (seed, sev)
        assert min(a[1], b[1]) > mc.SCAN_BARRIER_Y + 0.15, (seed, sev)       # free of the barrier
        d = (b - a) / np.linalg.norm(b - a)
        bare = -m3.tarp_side * np.array([-d[1], d[0]])          # messy_stand: struts and pavers on the bare side
        assert bare @ np.array([-1., 1.]) < 0, (seed, sev)      # team 2026-11-08: turned, its feet south-east
    m3 = _by_name(rebuilt[(0, 0.0)].walls)['M3']                 # Its pavers where both photos put them.
    a, b = (np.asarray(p) for p in m3.pieces[0])
    d = (b - a) / np.linalg.norm(b - a)
    out = np.array([-d[1], d[0]]) * mc.PAVER_OUT
    for measured, end in (((0.18, 0.97), a + d * 0.06), ((0.64, 1.62), b - d * 0.06)):
        assert np.linalg.norm(end + out - np.asarray(measured)) < 0.12, (end + out, measured)


def test_rebuild_north_west_panel_caps_the_first_corridor(rebuilt):
    """Team, 2026-11-08: NW is a short panel fully in the north-west corner, against the north net, its
    vinyl facing south (feet toward the net)."""
    for (seed, sev), c in rebuilt.items():
        nw = _by_name(c.walls)['NW']
        a, b = (np.asarray(p) for p in nw.pieces[0])
        d = b - a
        assert abs(_length(nw) - mc.SHORT_PANEL) < 0.06
        assert min(a[0], b[0]) < -mc.NET + 0.1                   # in the west corner
        assert min(a[1], b[1]) > mc.NET - 0.1                    # on the north net
        bare = -nw.tarp_side * np.array([-d[1], d[0]]) / np.linalg.norm(d)
        assert bare[1] > 0, (seed, sev)                          # feet toward the net, vinyl south


def test_rebuild_leaves_the_tape_and_checkpoint_lines(rebuilt):
    """The floor line and the lines tied to it (CP1, gate A) follow the walls
    as scanned: the same as the scan layout for the same draw. (The nets are
    drawn after the walls, so they differ between the two; the parts set from
    the nets, the CP1 line's y and west end and the tape's two CP1 vertices'
    y, are left out.)"""
    for seed, sev in CASES:
        if True:
            old = mc._sample(seed * 1000, sev, layout='scan')
            new = mc._sample(seed * 1000, sev, layout='rebuilt')
            g_old, g_new = np.asarray(old.guideline), np.asarray(new.guideline)
            assert np.allclose(g_old[:, 0], g_new[:, 0]), (seed, sev)
            keep = [k for k in range(len(g_old)) if k not in (1, 2)]
            assert np.allclose(g_old[keep, 1], g_new[keep, 1]), (seed, sev)
            assert np.isclose(old.lines['first_corridor']['end'][0], new.lines['first_corridor']['end'][0])
            for key in ('start', 'end', 'normal'):
                assert np.allclose(old.lines['gate_a'][key], new.lines['gate_a'][key]), (seed, sev, key)


def test_launch_spot_clear_of_stand_battens_at_floor_level():
    """Nothing lies under the drone's body and arms on the launch mark, down to the floor: sim
    seed 5 (severity 0.7, 2026-11-08) had the swung M1's stand frame 0.086 m from the launch,
    under the 3 cm floor band the check looked at, so the drone touched it at take-off."""
    from unittest import mock
    # The geometry it flew in: M1 at panel 1's end, no walls turned (team corrections came later).
    with mock.patch.object(mc, 'START_CORNER_GAP', 0.), \
            mock.patch.object(mc.Sampler, '_feet_toward', staticmethod(lambda wall, side: wall)):
        bad = mc._sample(5 * 1000 + 8, 0.7)      # the layout seed 5 flew (attempt 8)
    solid = {k: v for k, v in bad.groups.items() if not k.startswith(('Floor', 'Landing'))}
    grid, res, extent = mc._band_raster(solid, -0.005, 0.03)
    i = np.floor((np.asarray(bad.launch['xy']) + extent) / res).astype(int)
    r = int(math.ceil(mc.LAUNCH_FLOOR_CLEAR / res))
    window = grid[i[0] - r:i[0] + r + 1, i[1] - r:i[1] + r + 1]
    assert window[np.hypot(*np.mgrid[-r:r + 1, -r:r + 1]) * res <= mc.LAUNCH_FLOOR_CLEAR].any()   # it is there
    assert 'a tarp touches the drone on the launch mark' in mc.validate(bad)                          # and rejected


def test_every_rebuilt_course_validates_without_m2(rebuilt):
    """M2 gone and the NW panel added (photos/New_Walls_*), and the course still
    flyable. generate() validates before it adds the next course's frame outside
    the net (_surroundings), so check without that frame."""
    import copy
    for (seed, sev), c in rebuilt.items():
        names = {w.name for w in c.walls if not w.name.startswith('Barrier')}
        assert names == {'M1', 'M3', 'NW', 'C1', 'A1', 'C2a'}, (seed, sev)
        inside = copy.copy(c)
        inside.groups = {k: v for k, v in c.groups.items() if '(next course)' not in k}
        assert mc.validate(inside) == [], (seed, sev)


def test_barrier_panel_1_vinyl_faces_the_ladder(rebuilt):
    """Team, 2026-11-08: barrier panel 1 (behind the drone at takeoff) is turned: its vinyl faces the
    ladder (south), its stand's struts and feet face the launch corridor (north)."""
    for (seed, sev), course in rebuilt.items():
        b1 = _by_name(course.walls)['Barrier 1']
        a, b = (np.asarray(p) for p in b1.pieces[0])
        d = (b - a) / np.linalg.norm(b - a)
        assert (np.array([-d[1], d[0]]) * b1.tarp_side)[1] < 0, (seed, sev)


def test_gap_between_barrier_panel_1_and_the_start_corner(rebuilt):
    """Team drawing, 2026-11-08 (+y up): M1's base stands about 1 ft north of the barrier at the corner
    by the start and finish; panels 1 and 2 meet as before."""
    for (seed, sev), course in rebuilt.items():
        walls = _by_name(course.walls)
        a, b = (np.asarray(q) for q in walls['Barrier 1'].pieces[0])
        d = (b - a) / np.linalg.norm(b - a)
        base = np.asarray(walls['M1'].pieces[0][0])
        gap = float((base - b) @ np.array([-d[1], d[0]])) - mc.WALL_T / 2      # base to panel 1's face, north
        assert 0.25 <= gap <= 0.40, (seed, sev, gap)
        assert float(np.linalg.norm(np.asarray(walls['Barrier 2'].pieces[0][0]) - b)) < 0.05   # panels meet