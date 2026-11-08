# Cheese run (local simulation only)

A special route using the existing goal navigator: climb to 2.25 m, cross the
barrier, find the H mat through the colour pad detector, land and stay down.
The climb hands over to horizontal navigation only above 2.15 m. No checkpoint
maze, tunnel, window, or return is requested. The official referee still runs;
its course-completion result is independent of this experiment's landing result.

From `goalnav/`, with `localsim/requirements.txt` installed:

```sh
bash experiments/cheese/run.sh
```

Or provide the dependencies with Nix:

```sh
nix-shell -p 'python3.withPackages (p: [ p.numpy p.scipy p.pyyaml ])' --run 'bash experiments/cheese/run.sh'
```

Outputs: `localsim/out/cheese/seed1/viewer.html`, `trace.json`, `report.json`,
and synchronized images in `cam/`. Additional simulator arguments can be
passed to the script, including `--seed` and `--out` for a fresh run.

The course prior specifies a landing *search area*. The generated mat centre
is not passed to the navigator: the existing detector locates it in rendered
images. Simulation truth is used by the independent referee and replay marks.

Integrated inputs are the scan-aligned irregular course and boulders, CAD
body/rotor footprint, OAK-D pose from the drone TF (27.6 degree downward pitch),
4.5 Hz stereo with 130 ms latency and dropped matches, simulated PX4 odometry
with drift/noise, downward rangefinder, rough aerodynamics, and the unmodified
organizer flight interface. The replay also includes the phone scan overlay.

This explicitly enables the simulator's colour camera. The current drone
stack provides mono imagery only; this experiment does not establish that
it can find the mat on hardware. No drone is contacted, no PX4 settings are
changed, and the shortcut does not establish a valid scored course run.

For the compact standalone overview (sampled, compressed camera frames):

```sh
nix-shell -p 'python3.withPackages (p: [ p.pillow ])' --run 'python3 experiments/cheese/visualize.py localsim/out/cheese/seed1'
```

Open `cheese.html`. The 3D `viewer.html` retains all original camera frames
in its adjacent `cam/` folder. The compact overview embeds its images and
opens without a server.

Seed 1 / severity 0.7 result: DONE, disarmed on the mat; 0.055 m touchdown
error, zero contacts, no crash. The trace crosses the barrier at 2.259 m
and peaks at 2.276 m. Four recorded images have accepted pad detections.
This is one simulated run, not a robustness sweep or a hardware flight.
