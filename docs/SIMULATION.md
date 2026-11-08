# Current simulator guidance

From `goalnav/`, use the current goal navigator's simulator:

```sh
python3 -m localsim.goalrun --seed 1 --severity 0.7 --profile safe --duration 90 --out localsim/out/goalnav/s1
```

Install `localsim/requirements.txt`, or supply those dependencies with Nix.
The command normally writes `report.json`, `trace.json`, a 3D `viewer.html` and
camera frames. Keep the adjacent `cam/` directory with the viewer.

## Match the branch

- The current main adapter/simulator models CPU stereo points; use
  `--assume-colour-pad` to supply simulated colour to its pad detector.
- `agent/goalnav-oak-depth` changes the contract to depth Images and uses
  `--colour` instead. Its handoff records no simulation after those changes.
- `origin/agent/sim-walls` contains rebuilt-maze/ladder geometry. The older
  scan layout includes walls that have since moved or been removed. Even the
  rebuilt version retains unresolved floor-tape/checkpoint placement.

Run `python3 -m localsim.goalrun --help` on the checkout being used. Record the
commit, layout, seed, sensor flags and tuning with a result; do not transfer
results between these versions as if they were the same experiment.

## What a successful run proves

The goalrun path executes the goal navigator and organizer flight-interface
code against fake ROS and a Python vehicle model. It models noisy sensing,
odometry, geometry and rough dynamics. It does **not** run the real camera,
cuVSLAM, DDS transport or PX4 firmware. Simulated depth and colour are rendered;
the referee and replay see ground truth independently of the navigator.

The one local cheese run (`fd96af8`) used scan-era geometry, a 2.25 m fixed climb,
a known landing search area and rendered colour. It reported a 5.5 cm landing
error and zero contacts. It did not validate the rebuilt course, actual colour
pipeline, real mat detector or hardware launch. It skipped the course checkpoints.

The former race stack and its guide (`goalnav/docs/LOCAL_SIM.md`) were removed on
2026-11-07: no race or fast modes, we play it safe. `localsim/run.py --px4` remains
for PX4 SITL.
[Organizer AI_CONTEXT.md](../goalnav/docs/organizer/AI_CONTEXT.md) describes a
separate lab simulator; its ROS version, topics and arming example are not the
hardware deployment contract. [The Isaac/Pegasus spike](../skybetav1/sim/README.md)
is another separate path and is documented as unexecuted.
