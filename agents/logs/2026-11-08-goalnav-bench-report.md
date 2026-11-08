# goalnav on skybeta-03: findings report (2026-11-07 21:50 to 2026-11-08 01:35)

Sessions: claude with James. Simulator work in `goalnav/localsim`; drone work over USB
(`192.168.55.1`), props off throughout. Nothing was flown.

## Summary

- **The camera has to be `RGBSTEREO`** (colour + mono pair, depth computed on the Jetson). On-camera
  depth (`rgbd`) gives colour and depth, but PX4 stops fusing visual odometry, so it has no position.
- **goalnav's code runs on the drone:** it deploys, builds and starts with the RGBSTEREO image, and
  the Jetson depth is fast enough (5–8 Hz).
- **goalnav is NO-GO as configured.** With goalnav's stack running (goalnav, its stereo node, the
  flight-log recorder), visual odometry reaches PX4 236–400 ms old, and PX4 fuses it only about 60 %
  of the time. Without goalnav (`overwall_nav`, or the bare stack) the same drone gives 62–105 ms,
  fused. The cause is CPU load on the Jetson.
- **In the simulator** at conservative drone numbers, the full course (near ladder) finishes and
  returns to the launch with no contacts. One run did touch a wall on the return; the referee missed
  it (bug below).
- **Next:** cut goalnav's CPU load, then warm up off the pad and run a 30 s check there before taking
  the pad.

## 1. Camera pipeline: on-camera depth vs RGBSTEREO

| Pipeline | Colour | Depth | Mono pair | cuVSLAM | Vision age at PX4 | PX4 fusion (`cs_ev_pos`, `xy_valid`) |
|---|---|---|---|---|---|---|
| `rgbd` (on-camera depth), 23:40–23:49 | 4.7–9.3 Hz | 25 Hz on camera | **11–19 Hz** (wants 30) | 19–25 Hz | **665 ms** | **false**, dead reckoning |
| `RGBSTEREO`, stock stack + overwall_nav, 00:26 | 9.1 Hz | 3.15 Hz CPU (overwall `stereo_depth`), 88–400 ms | 29 Hz | 31–32 Hz | **62–105 ms** | **true** |

- The team found the same with plain `depth` earlier (290 ms, not fused; `agents/logs/2026-11-07-overwall-pad.md`).
  On-camera depth overloads the OAK-D's USB link, and the mono pair drops frames.
- Decision: `RGBSTEREO`, with goalnav's own `stereo_node` making depth on the Jetson (main, PR #21/#24).
- The colour camera's tilt is fixed by the mount at 27.0° (TF, `oak_rgb_camera_optical_frame`). The
  left mono camera is at 27.6°.

## 2. goalnav bench on the drone (2026-11-08 01:15–01:33)

Deployed `main` (goalnav with `camera:=rgbstereo`, safe profile 0.4 m/s), built OK, mission
`drone_ref / goalnav.launch.py`.

| Time after boot | Video in bag | Jetson depth | Colour | cuVSLAM | Vision age | PX4 fused | Verdict |
|---|---|---|---|---|---|---|---|
| 1 min | yes | 5.2 Hz | 7.2 Hz | 29 Hz | 354 ms (snapshot) | yes (snapshot) | NO-GO |
| 2 min | yes | — | — | — | 60–77 ms (5 snapshots) | yes | — |
| 3 min | yes | 5.0 Hz | 7.1 Hz | 27 Hz | 172 ms | **no** (snapshot) | NO-GO |
| 5 min | yes | — | — | — | 262–404 ms (6 snapshots) | 18/18 over 20 s | — |
| 1 min (after reboot) | **no** | 7.8 Hz | **3.5 Hz** | 31.7 Hz | **median 236 ms, p90 351 ms** (30 s) | **60 % of 115 samples** (30 s) | NO-GO |

CPU at 5 min with video (`top`): camera/cuVSLAM containers 106 % + 50 %, bag recorder 44 %, goalnav
44 %, flight_interface 31 %, stereo_node 31 %.

What passed every time: mission, camera `RGBSTEREO`, goalnav + stereo_node + flight_interface
running, PX4 link, goalnav status `IDLE`, flight log recording, PX4 `xy_valid` at the moment of reading.

### Why it fails

Visual odometry reaches PX4 late, and the delay comes and goes. PX4 drops vision that arrives too late,
so fusion flickers. The same drone without goalnav's stack fuses with 62–105 ms vision, so the cause is
the load goalnav adds (goalnav, stereo_node, the bag recorder) on the cores cuVSLAM and the odometry
bridge need. Turning off mono video in the bag did not fix it. The early readings were also taken
1–5 min after boot, when everything runs slow.

### Flying slowly does not fix it

At 0.4 m/s, 0.4 s-old vision is 16 cm of travel; speed is not the problem. The danger is a fusion
dropout in flight: PX4 loses its position, Position mode stops holding, and the pilot has to take over.

## 3. Changes made on the drone tonight

- Camera image: built `rgbd` to test it, then restored `RGBSTEREO` (backup on the drone:
  `~/team_ws/parked/oakd_vslam.yaml.rgbstereo-2337`). It runs RGBSTEREO now.
- goalnav from `main` deployed to `~/team_ws/src/drone_ref` (previous copy kept as `~/drone_ref.prev-*`).
- **Uncommitted, drone only:** `record_video` default set to `false` in the drone's
  `goalnav.launch.py`. The same edit is in the worktree `SC-novideo`; it is not on `main`.
- Mission selected: `drone_ref / goalnav.launch.py`. **The drone lock is held by claude (James)** until released.
- The Jetson was rebooted by command three times (no stack restart otherwise). The PX4 link survived
  Jetson-only reboots. After a flight-controller power-cycle it needed the flight battery on the main lead.

Operational notes:
- The Jetson does **not** reboot when the flight battery is cycled while it has other power. A new
  mission selection needs a Jetson reboot (`systemctl reboot`) to take effect.
- The PX4 link was down whenever the flight controller had no battery power. With the battery in, it
  came up within about a minute.

## 4. Simulator findings

All at conservative numbers: depth and colour 2.8 Hz, 0.4 s old (measured 3–8 Hz on the drone),
colour camera 27.0°, safe profile 0.4 m/s, near ladder within 1.5 ft of the barrier, rocks at least
1.5 ft from the mat's edge.

| Run | Result | Notes |
|---|---|---|
| Full course, near ladder, seed 405 (v9) | Done, returned, 0 contacts | 0.85 s on the pad floor; rotor tips 0.21 m from the west net, 0.38 m from the barrier centreline |
| Full course, near ladder, seeds 601–603 (old course) | 601: done, returned; **602: done, but overlapped a wall by 9.5 cm** on `RETURN_DESCEND` at (-2.47, 0.59, 0.81), t = 78.2 s; 603: stopped | Old geometry (before the corner change) |

### Fixed this session (merged to main, PR #24/#25)

- The pad rest now starts from goalnav's own touchdown (height and vertical speed), not PX4's land
  detector (which took ~3.4 s). Total ~0.85 s on the floor, so it climbs before PX4's 2 s auto-disarm.
- Return spot: 0.725 m off the barrier (the props had been over the stand's feet) and halfway between
  the west net and M1 (0.70 m).
- Sim: rocks never on the mat; the near ladder corrected; colour tilt from the TF.
- Removed: the cheese shortcut and the `fast`/`normal` profiles.
- Tools: `tools/fly bench`, `tools/postflight.py`, `tools/drone` over USB.

### Open, not fixed

1. **Referee misses slow contacts after the pad.** In `localsim/referee.py`, `_update_return` only
   acts on contacts above crash speed (1 m/s), so seed 602's 9.5 cm overlap counted 0 touches. Any
   contact on the return should be a touch and fail the return.
2. **The return started its descent too close to the barrier** (seed 602: 0.24 m from its centreline).
   The descent needs to wait until the drone is clear of the barrier (the homeward descent gate is
   currently off).
3. **Course change requested, not modelled:** a 2 ft gap at the landing corner (M1 / barrier panel 1),
   and panel 1 turned so its vinyl faces the ladder. My first attempt (M1 moved 2 ft north) closes the
   maze passage north of M1, so the real geometry is different. **Which panel moved, and which way?**
   With panel 1 turned, its stand's struts and feet face the launch corridor and reach the drone's
   footprint on the launch mark in some layouts: **check the prop clearance at the launch mark on the
   real course.** Work in progress (uncommitted): worktree `SC-corner`.

## 5. Tool changes in progress (uncommitted, worktree `SC-novideo`)

- `tools/bench-goalnav.sh`: PX4 fusion and vision age judged over **30 s** (want fusion ≥ 95 % and
  median vision age < 150 ms), plus `xy_valid`, instead of single snapshots.
- `goalnav/launch/goalnav.launch.py`: `record_video` default `false`.

## 6. Proposal

1. **Give the pad back now:** release the lock and select `overwall_nav` again.
2. **Cut goalnav's CPU load (off the drone):**
   - stereo_node `rate_hz` 10 → 4 (it ran 5–8 Hz; the sim needs 2.8);
   - goalnav `control_hz` 50 → 20;
   - stop recording raw colour images in the bag (keep PX4, odometry, goalnav status, depth points);
   - add CPU per process to the bench check.
3. **Next drone slot, without holding the pad:** power on at the side and warm up there, run the 30 s
   bench check, carry it to the pad only once it passes, and re-check fusion for 30 s on the pad.
   If it still fails: stereo_node at lower resolution.
4. Fix the referee (contacts on the return) and the return descent gate, with evals; model the corner
   change once its geometry is known; re-run the near-ladder full course one run at a time.

## 7. Not measured

- No flight with goalnav. The real mat detector has never seen the real mat.
- PX4's real land-detector timing.
- `tools/postflight.py` on a real flight log (only a synthetic one).
- goalnav's stack after a full 5–10 min warm-up: the last check was 1 min after boot.
