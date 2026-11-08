"""tools/manual-flight: set up, check, pull and undo a hand-flown, fully recorded flight.

Runs the real script against a fake drone (a local folder standing in for the drone's home, a fake
docker for the stack container, a fake lock), so no hardware is needed. What must hold:
- setup selects the recorder-only mission and remembers the one it replaced (once, not overwritten);
- check says GO only with the recorder running, the log growing, and NOTHING that can fly the drone;
- pull stops the recorder first (a closed bag), then copies the flight complete;
- restore puts the remembered mission back and releases the lock.

    python -m pytest tools/test_manual_flight.py        (needs bash: Git Bash on Windows)
"""
from pathlib import Path
import os
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
BASH = shutil.which('bash')
pytestmark = pytest.mark.skipif(BASH is None, reason='needs bash')

# A fake tools/drone: "the drone" is $FAKE_HOME here; /etc/skybeta/drone.env is $FAKE_HOME/drone.env;
# `docker exec skybeta-stack ...` goes to the fake docker below. Every command is logged to $FAKE_HOME/calls.
DRONE = r'''#!/usr/bin/env bash
cmd="$1"; shift
printf '%s %s\n' "$cmd" "$*" >> "$FAKE_HOME/calls"
case "$cmd" in
  run|runt)
    c="$*"
    if [ "$c" = hostname ]; then echo skybeta-03; exit 0; fi
    case "$c" in *"du -sk"*) for b in "$FAKE_HOME"/team_ws/logs/*/bag; do [ -f "$FAKE_HOME/recording" ] && head -c 9000 /dev/zero >> "$b/grow"; done ;; esac
    c="${c//\/etc\/skybeta\/drone.env/$FAKE_HOME/drone.env}"
    bin="$(cygpath -u "$FAKE_BIN" 2>/dev/null || printf '%s' "$FAKE_BIN")"     # Git Bash: C:/x breaks PATH
    HOME="$FAKE_HOME" PATH="$bin:$PATH" bash -c "$c" ;;
  push) dst="${2/#\~/$FAKE_HOME}"; cp -r "$1" "$dst" ;;
  pull) src="${1/#\~/$FAKE_HOME}"; cp -r "$src" "$2" ;;
  *) echo "stub: $cmd not supported" >&2; exit 9 ;;
esac
'''
# The stack container: the recorder is "running" while $FAKE_HOME/recording exists; $FAKE_HOME/procs lists
# other processes (e.g. a flight_interface). pkill -INT closes the bag as rosbag2 does (metadata.yaml).
DOCKER = r'''#!/usr/bin/env bash
[ "$1 $2" = "exec skybeta-stack" ] || { echo "fake docker: $*" >&2; exit 9; }
shift 2
case "$1" in
  pgrep)
    if [ "$2" = -f ] && [ "$3" = "ros2 bag record" ]; then [ -f "$FAKE_HOME/recording" ]; exit $?; fi
    if [ "$2" = -af ]; then grep -E "$3" "$FAKE_HOME/procs" 2>/dev/null; exit $?; fi ;;
  pkill)
    if [ -f "$FAKE_HOME/recording" ]; then
      for b in "$FAKE_HOME"/team_ws/logs/*/bag; do echo "closed" > "$b/metadata.yaml"; done
      rm -f "$FAKE_HOME/recording"
    fi; exit 0 ;;
  true) [ ! -f "$FAKE_HOME/stack_down" ]; exit $? ;;
  bash) echo "fake build ok"; exit 0 ;;
esac
echo "fake docker: unexpected $*" >&2; exit 9
'''
RUN_MISSION = r'''#!/usr/bin/env bash
sed -i "s/^COMPETITOR_PKG=.*/COMPETITOR_PKG=$1/; s/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=$2/" "$HOME/drone.env"
'''
LOCK = r'''#!/usr/bin/env bash
case "$1" in acquire) echo "$2" > "$FAKE_HOME/lock"; echo acquired ;; release) rm -f "$FAKE_HOME/lock"; echo released ;; esac
'''


def _script(path, text):
    path.write_text(text, newline='\n')
    path.chmod(0o755)
    return path


@pytest.fixture
def drone(tmp_path):
    home = tmp_path / 'drone_home'
    (home / 'team_ws' / 'logs').mkdir(parents=True)
    (home / 'team_ws' / 'src').mkdir()
    (home / 'skybeta' / 'host').mkdir(parents=True)
    _script(home / 'skybeta' / 'host' / 'doctor.sh', '#!/usr/bin/env bash\necho "doctor: all PASS"\n')
    _script(home / 'team_ws' / 'run_mission.sh', RUN_MISSION)
    (home / 'drone.env').write_text('ROS_DOMAIN_ID=0\nCOMPETITOR_PKG=drone_ref\nCOMPETITOR_LAUNCH=goalnav.launch.py\n',
                                    newline='\n')
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    _script(fake_bin / 'docker', DOCKER)
    stub = _script(tmp_path / 'drone_stub', DRONE)
    lock = _script(tmp_path / 'lock_stub', LOCK)
    out = tmp_path / 'laptop_logs'

    def manual(*args):
        env = {'PATH': os.environ['PATH'], 'FAKE_HOME': home.as_posix(), 'FAKE_BIN': fake_bin.as_posix(),
               'FLY_DRONE': str(stub), 'FLY_LOCK': str(lock), 'FLY_LOGS_DIR': str(out),
               'DRONE_PASS': 'unused', 'FLY_NO_SUMMARY': '1'}
        return subprocess.run([BASH, str(ROOT / 'tools' / 'manual-flight'), *args], env=env,
                              capture_output=True, text=True, timeout=90)
    return home, out, manual


def _env(home):
    return dict(line.split('=', 1) for line in (home / 'drone.env').read_text().splitlines() if '=' in line)


def _powered_up(home, name='20261108-120000'):
    """What a power-up with manual.launch.py leaves: a run folder with an open bag, the recorder running."""
    run = home / 'team_ws' / 'logs' / name
    (run / 'bag').mkdir(parents=True)
    (run / 'bag' / 'bag_0.mcap').write_bytes(b'x' * 5000)
    (run / 'run.json').write_text('{"mission": "manual_flight"}')
    (home / 'recording').write_text('')
    return run


def test_setup_selects_the_recorder_and_remembers_the_mission_it_replaced(drone):
    home, _, manual = drone
    r = manual('setup', 'Tester')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _env(home)['COMPETITOR_PKG'] == 'manual_flight'
    assert _env(home)['COMPETITOR_LAUNCH'] == 'manual.launch.py'
    assert 'COMPETITOR_PKG=drone_ref' in (home / 'team_ws' / '.mission-before-manual').read_text()
    assert (home / 'team_ws' / 'src' / 'manual_flight' / 'launch' / 'manual.launch.py').is_file()
    assert (home / 'lock').read_text().strip() == 'Tester'
    assert 'power-cycle' in r.stdout


def test_setup_twice_keeps_the_real_previous_mission(drone):
    home, _, manual = drone
    manual('setup', 'Tester')
    r = manual('setup', 'Tester', '--no-video')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _env(home)['COMPETITOR_LAUNCH'] == 'manual_novideo.launch.py'
    assert 'COMPETITOR_PKG=drone_ref' in (home / 'team_ws' / '.mission-before-manual').read_text()


def test_check_go_when_recording_and_nothing_can_fly_the_drone(drone):
    home, _, manual = drone
    manual('setup', 'Tester')
    _powered_up(home)
    r = manual('check')
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'GO: arm and fly by hand.' in r.stdout
    assert 'FAIL' not in r.stdout


def test_check_no_go_when_an_autopilot_is_running(drone):
    # The danger: flight_interface takes the drone on arm + Position.
    home, _, manual = drone
    manual('setup', 'Tester')
    _powered_up(home)
    (home / 'procs').write_text('412 /usr/bin/python3 /root/skybeta_ws/install/lib/flight_interface\n')
    r = manual('check')
    assert r.returncode != 0
    assert 'NO-GO' in r.stdout and 'flight_interface' in r.stdout


def test_check_no_go_when_it_cannot_see_into_the_stack(drone):
    # No container = cannot tell what runs: never report "no autopilot" on a failed look.
    home, _, manual = drone
    manual('setup', 'Tester')
    _powered_up(home)
    (home / 'stack_down').write_text('')
    r = manual('check')
    assert r.returncode != 0
    assert 'no autopilot running' not in r.stdout
    assert 'stack container' in r.stdout


def test_check_no_go_when_the_old_mission_is_still_selected(drone):
    home, _, manual = drone
    _powered_up(home)
    r = manual('check')
    assert r.returncode != 0
    assert "mission is 'drone_ref'" in r.stdout


def test_check_no_go_when_the_recorder_is_not_running(drone):
    home, _, manual = drone
    manual('setup', 'Tester')
    _powered_up(home)
    (home / 'recording').unlink()
    r = manual('check')
    assert r.returncode != 0
    assert 'recorder not running' in r.stdout


def test_pull_closes_the_bag_then_copies_the_flight_complete(drone):
    home, out, manual = drone
    manual('setup', 'Tester')
    _powered_up(home)
    r = manual('pull')
    assert r.returncode == 0, r.stdout + r.stderr
    assert not (home / 'recording').exists()                                   # recorder stopped first
    got = out / '20261108-120000'
    assert (got / 'bag' / 'metadata.yaml').read_text().strip() == 'closed'    # the closed bag came over
    assert (got / 'bag' / 'bag_0.mcap').stat().st_size == 5000
    assert 'complete' in r.stdout.lower()
    calls = (home / 'calls').read_text()
    assert calls.index('pkill -INT') < calls.index('pull ~/team_ws/logs/20261108-120000')


def test_pull_refuses_while_another_mission_is_selected(drone):
    # Never stop a mission's recorder (tools/fly logs is for those).
    home, _, manual = drone
    _powered_up(home)
    r = manual('pull')
    assert r.returncode == 2
    assert (home / 'recording').exists()


def test_restore_puts_back_the_remembered_mission_and_releases_the_lock(drone):
    home, _, manual = drone
    manual('setup', 'Tester')
    r = manual('restore')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _env(home)['COMPETITOR_PKG'] == 'drone_ref'
    assert _env(home)['COMPETITOR_LAUNCH'] == 'goalnav.launch.py'
    assert not (home / 'lock').exists()
    assert not (home / 'team_ws' / '.mission-before-manual').exists()


def test_restore_with_nothing_remembered_goes_back_to_overwall(drone):
    home, _, manual = drone
    (home / 'drone.env').write_text('COMPETITOR_PKG=manual_flight\nCOMPETITOR_LAUNCH=manual.launch.py\n')
    r = manual('restore')
    assert r.returncode == 0, r.stdout + r.stderr
    assert _env(home)['COMPETITOR_PKG'] == 'overwall_nav'
