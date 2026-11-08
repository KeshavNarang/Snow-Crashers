"""tools/fly logs really lands a flight's log folder on the laptop, complete, and only once.

The team's ask (2026-11-08): data from failed flights was only on the drone; pulling it must be quick
and dependable. These run tools/fly against a fake drone (a local folder standing in for the drone's
~/team_ws/logs, through FLY_DRONE), so no hardware is needed.

    python -m pytest tools/test_fly_logs.py        (needs bash: Git Bash on Windows)
"""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
BASH = shutil.which('bash')
pytestmark = pytest.mark.skipif(BASH is None, reason='needs bash')

STUB = r'''#!/usr/bin/env bash
# A fake tools/drone: "the drone" is $FAKE_HOME on this machine.
cmd="$1"; shift
case "$cmd" in
  run)  if [ "$*" = hostname ]; then echo skybeta-03; else HOME="$FAKE_HOME" bash -c "$*"; fi ;;
  pull) src="${1/#\~/$FAKE_HOME}"; cp -r "$src" "$2" ;;
  *) echo "stub: $cmd not supported" >&2; exit 9 ;;
esac
'''


def _flight(home, name, files):
    run = home / 'team_ws' / 'logs' / name
    for rel, size in files.items():
        p = run / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(b'x' * size)
    return run


@pytest.fixture
def drone(tmp_path):
    home = tmp_path / 'drone_home'
    (home / 'team_ws' / 'logs').mkdir(parents=True)
    stub = tmp_path / 'drone_stub'
    stub.write_text(STUB, newline='\n')
    out = tmp_path / 'laptop_logs'

    def fly(*args):
        env = {'PATH': subprocess.os.environ['PATH'], 'FAKE_HOME': str(home), 'FLY_DRONE': str(stub),
               'FLY_LOGS_DIR': str(out), 'DRONE_PASS': 'unused', 'FLY_NO_SUMMARY': '1'}
        return subprocess.run([BASH, str(ROOT / 'tools' / 'fly'), 'logs', *args], env=env,
                              capture_output=True, text=True, timeout=60)
    return home, out, fly


def test_newest_flight_lands_complete_on_the_laptop(drone):
    home, out, fly = drone
    _flight(home, '20261108-100000', {'run.json': 50, 'bag/flight_0.mcap': 4000})
    import os, time
    newer = _flight(home, '20261108-110000', {'run.json': 60, 'bag/flight_0.mcap': 9000, 'ros/goalnav.log': 300})
    t = time.time() + 10
    os.utime(newer, (t, t))
    r = fly()
    assert r.returncode == 0, r.stdout + r.stderr
    got = out / '20261108-110000'
    assert (got / 'bag' / 'flight_0.mcap').stat().st_size == 9000
    assert (got / 'ros' / 'goalnav.log').stat().st_size == 300
    assert 'complete' in r.stdout.lower()
    assert not (out / '20261108-100000').exists()


def test_a_named_older_flight_can_be_pulled(drone):
    home, out, fly = drone
    _flight(home, '20261108-100000', {'run.json': 50})
    _flight(home, '20261108-110000', {'run.json': 60})
    r = fly('20261108-100000')
    assert r.returncode == 0, r.stdout + r.stderr
    assert (out / '20261108-100000' / 'run.json').stat().st_size == 50


def test_second_pull_is_skipped_and_partial_copy_is_redone(drone):
    home, out, fly = drone
    _flight(home, '20261108-120000', {'run.json': 10, 'bag/b.mcap': 500})
    assert fly().returncode == 0
    r = fly()
    assert r.returncode == 0 and 'already' in r.stdout.lower(), r.stdout
    (out / '20261108-120000' / 'bag' / 'b.mcap').write_bytes(b'x' * 100)      # a cut-short copy
    r = fly()
    assert r.returncode == 0, r.stdout + r.stderr
    assert (out / '20261108-120000' / 'bag' / 'b.mcap').stat().st_size == 500


def test_no_flights_is_a_clear_error_and_copies_nothing(drone):
    home, out, fly = drone
    r = fly()
    assert r.returncode != 0
    assert 'no flight logs' in (r.stdout + r.stderr).lower()
    assert not out.exists() or not any(out.iterdir())


def test_list_shows_recent_flights_and_skips_overwall(drone):
    home, out, fly = drone
    _flight(home, '20261108-130000', {'run.json': 10})
    (home / 'team_ws' / 'logs' / 'overwall').mkdir()
    r = fly('list')
    assert r.returncode == 0, r.stdout + r.stderr
    assert '20261108-130000' in r.stdout and 'overwall' not in r.stdout
    assert not out.exists() or not any(out.iterdir())
