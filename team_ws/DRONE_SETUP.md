# human notes:

> Current cross-project guidance: [docs/FLIGHT.md](../docs/FLIGHT.md) and [docs/STATUS.md](../docs/STATUS.md). The user confirms several successful simple up-and-move flights. Historical boot state, parameter issues and restrictions below are not a live inspection.
code stored in: cd ~/team_ws/src
Run to set the flags:  ~/team_ws/run_mission.sh snowcrashers_nav low_test.launch.py

# Fly our code on skybeta-03

## TL;DR

**Code:** `~/team_ws/src/snowcrashers_nav` on the drone (copy of `team_ws/src/snowcrashers_nav` in this repo).
**Launch file:** `low_test.launch.py` (0.5 m up, 0.5 m forward) or `competitor.launch.py` (2 m / 2 m),
chosen in `/etc/skybeta/drone.env`. The drone builds and launches it by itself at power-up.

After `ssh dronehacks@skybeta-03.local` (`hostname` must say `skybeta-03`), with the drone on the
floor, disarmed and still:

```bash
~/team_ws/run_mission.sh snowcrashers_nav low_test.launch.py      # or competitor.launch.py for 2 m
```

It only sets which package and launch file run (and warns if autostart is off). It does not
start the stack: with autostart on, every power-up runs whatever was set last. Only needed when
you change the launch file. Then battery out 10 s, in, wait about 3 minutes, and:

1. Mode switch to **Altitude**, then check PX4 actually sees it (must print `ALTCTL`):
   `docker exec skybeta-stack tmux capture-pane -p -t skybeta:competitor | grep nav_state | tail -1`
2. **Arm**, then **Position within 3 s**. It flies, lands and disarms by itself.
   Take over at any time: **throttle to the middle**.

Never change PX4 parameters. Anything not working: see [What we learned](#what-we-learned-on-skybeta-03-2026-11-07)
and the troubleshooting table below.

---

## What we learned on skybeta-03 (2026-11-07)

It flew on the first attempt once these were right:

- **Autostart was initially absent; later sessions record it enabled.** Check the live service. `run_mission.sh` only selects the mission; `sudo host/autostart.sh enable` separately installs/enables and starts the service.
- **Arming alone never starts the code. The switch to Position after arming does.** The
  sequence is Altitude, arm, Position within 3 s. Arming with the switch already on Position,
  or in Manual without the switch change, gives no autonomy.
- **Check PX4 sees the mode switch before arming.** On a failed attempt PX4 had stayed in
  `MANUAL` the whole time, so the switch to Position never reached it. The `nav_state` line must
  show `ALTCTL` with the switch on Altitude.
- **`uart-traffic FAIL` in `host/doctor.sh` means the flight controller is silent.** Unplugging
  and replugging both batteries fixed it.
- **The stack is slow to start:** up to about 3 minutes after power-up (it waits for the clock to
  sync). Wait for `PX4 link up` before arming.
- **Container screen history may be lost on restart.** Read the `auto offboard:` lines before cycling power. Goalnav's recorder writes persistent files under the mounted `~/team_ws/logs/`; those survive container recreation.
- **Check `drone.env` before each session.** It had been switched back to the other launch file
  between attempts. `grep COMPETITOR /etc/skybeta/drone.env` shows what will run.
- **Known issue on the 0.5 m test:** PX4's height estimate read 0.3 to 0.45 m below the camera's
  on the ground. The flight interface's 0.3 m floor (measured on the camera) then keeps every
  target at about 0.6 to 0.75 m above the floor, so the low test climbs there, waits about 16 s,
  and lands in place without the forward leg. Safe, but not the full test. The 2 m mission is
  not affected. A code fix is pending.
- **Keep the default PX4 parameters** (organisers' instruction). Our code needs none changed.

---

## Step by step

**What it does:** the pilot arms the drone and switches to Position. The drone then climbs
2 m, flies 2 m forward (the way its nose points), lands and turns its motors off by itself.

Do every step in order. Type the commands exactly. Each step says what you should see.
If you don't see it, stop and look in [Something went wrong](#something-went-wrong).

---

## Never do these

- **Never change PX4 parameters** (no QGroundControl parameter edits, no `.params` files, no AI
  "tuning"). The organisers: *almost no one with modified parameters has been able to fly.* Our
  code needs **no** parameter changes. If anyone changed this drone's parameters before, tell
  Kelvin (Slack, kelvinkn@stanford.edu) and ask him to put the defaults back. Ignore the
  parameter changes suggested in `reference_stack/docs/JETSON.md`.
- Never SSH into a drone that isn't **skybeta-03** (disqualification).
- Never change other lines of `/etc/skybeta/drone.env`, the password, `host/doctor.sh` or the
  monitoring services.
- Never press **Ctrl-C** inside the drone's tmux view. It stops our code. Leave with **Ctrl-b**,
  then **d**.

## What you need

- A laptop on the **DroneHacks** Wi-Fi.
- The drone login credentials from the local `.secrets/skybeta-03.env` file; do not copy them into documentation.
- A charged battery.
- A **safety pilot** with the RC transmitter, who knows the kill switch (channel 5).
- For real flights: props, good light, a floor with texture (not plain white), and clear space:
  1 m ahead for the first flight, 3 m ahead and 2.5 m overhead for the full one.

---

## Part A: one-time setup (about 10 minutes)

### A1. Power on

Put the battery in. Put the drone on the floor where it will take off, nose pointing the way it
should fly. **Don't touch it for 1 minute.** The camera sets its zero point at power-up.

### A2. Log in

On your laptop, open a terminal:

```bash
ssh dronehacks@skybeta-03.local
```

Type the password. The first time, answer `yes` to the fingerprint question.
If it times out: check the Wi-Fi, or plug a USB-C cable into the Jetson and use
`ssh dronehacks@192.168.55.1`.

Then:

```bash
hostname
```

✅ It prints `skybeta-03`. **Anything else: type `exit` straight away. That's not our drone.**

### A3. Check the software and our code are there

```bash
cat ~/skybeta/VERSION
ls ~/team_ws/src
```

✅ The version is `0.3.3` or higher. **0.3.2 or lower cannot take over at all: ask Kelvin to
update it.**
✅ The list shows `snowcrashers_nav`.

If the list also shows something else, move it out, because one broken package stops everything
from starting. Replace `NAME` with what it shows:

```bash
mkdir -p ~/team_ws/parked && mv ~/team_ws/src/NAME ~/team_ws/parked/
```

### A4. Health check

```bash
cd ~/skybeta && host/doctor.sh
```

✅ The line `uart-traffic` says **PASS**. That means the flight controller is talking to the Jetson.

❌ **FAIL**: take the battery out, wait 10 seconds, put it back, wait 1 minute, then log in again
(A2) and re-run. Still FAIL: check the lights on the Pixhawk are on (it has power), then message
Kelvin. Nothing will fly until this passes.

(Other WARN lines are usually fine. FAIL on anything else: message Kelvin.)

### A5. Tell the drone to run our code

This command changes only the two lines that teams are allowed to change. Copy it whole:

```bash
sudo sed -i 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=snowcrashers_nav/; s/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=low_test.launch.py/' /etc/skybeta/drone.env
grep COMPETITOR /etc/skybeta/drone.env
```

`sudo` asks for the password again: it's the same one. ✅ You see exactly:

```
COMPETITOR_PKG=snowcrashers_nav
COMPETITOR_LAUNCH=low_test.launch.py
```

`low_test` is the small version (0.5 m up, 0.5 m forward) for the first flights. Part D
switches to the full 2 m.

### A6. Make it start by itself at every power-up

On this drone the software doesn't start by itself yet. This installs the organisers' start-up
service and starts it now. **The drone must be on the floor and still.**

```bash
cd ~/skybeta && sudo host/autostart.sh enable
```

Wait 30 seconds, then:

```bash
host/autostart.sh status
```

✅ It says `active`.

### A7. Check our code is running

```bash
docker exec skybeta-stack tmux capture-pane -p -S -300 -t skybeta:competitor | grep -E "team_build|Offboard on the switch|PX4 link up|up_forward_land ready|build failed"
```

✅ All four appear:

```
[team_build] OK ...
  Offboard on the switch to Position after arming: ON
... PX4 link up: ...
... up_forward_land ready: up 0.5 m, forward 0.5 m, land ...
```

Then check the drone knows where it is:

```bash
docker exec skybeta-stack bash -c 'source skybeta_env.sh >/dev/null 2>&1; timeout 8 ros2 topic echo --once /skybeta/state'
```

✅ The output contains `"px4_link": true` and `"local_enu": [` followed by numbers (not
`null`). `null` means vision isn't tracking yet: wait a minute and try again.

**Setup is done.** From now on, every power-up starts our code by itself.

---

## Part B: rehearsal with the props OFF (do this first, and after every code change)

1. **Take the props off.** Unplug the Pixhawk's USB cable if one is plugged in: PX4 refuses to arm
   with USB connected.
2. Battery out, 10 seconds, battery in. Drone on the floor, don't touch it for 1 minute.
3. Log in (A2) and run the A7 checks.
4. Open the live view:

   ```bash
   docker exec -it skybeta-stack tmux attach -t skybeta
   ```

   Press **Ctrl-b**, then **3**: this is the `competitor` window, where our code prints.
   (To leave: **Ctrl-b**, then **d**. **Never Ctrl-C here.**)
5. Pilot: mode switch on **Altitude**, throttle **down**, **arm**. **Within 3 seconds**, switch
   to **Position**.
6. ✅ In the window:

   ```
   auto offboard: switched to Position after arming: requesting Offboard
   [CLIMB] Offboard active: start (...), heading ... Climbing to 0.5 m.
   ```

   The motors speed up. With no props it can't lift, so about 10 seconds after arming PX4 turns
   the motors off by itself. That's expected. Then:
   `[ABORTED] Offboard ended ...` and, a few seconds later,
   `[WAIT] Disarmed: ready for the next flight`.
7. Pilot: mode switch back to **Altitude**.

If you saw `auto offboard: staying out of Offboard: <reason>` instead, look up the reason in
[Something went wrong](#something-went-wrong).

---

## Part C: first real flight (0.5 m up, 0.5 m forward)

1. **Props on.** Clear space: 1 m ahead of the nose, nobody near the drone.
2. Battery out, 10 seconds, battery in. Drone on its takeoff spot, nose pointing where it should
   fly. **Don't touch it for 1 minute.**
3. Log in (A2) and run the A7 checks. Open the live view (B4) if you want to watch.
4. Safety pilot: transmitter on, kill switch checked, standing clear.
5. Pilot: **Altitude**, throttle **down**, **arm**, then **within 3 seconds** switch to
   **Position**.
6. ✅ The drone climbs to 0.5 m, pauses, moves 0.5 m forward, pauses, comes down, lands, and the
   motors stop 2 to 4 seconds after touchdown. The window ends with
   `[DONE] Disarmed. RESULT {...}`.
7. **To take over at any moment: raise the throttle to the middle** (or move any stick). The
   drone then holds in Position under the pilot, who lands it. Emergency only: the kill switch
   (motors stop, it falls). Don't just flip the mode switch with the throttle low: it will sink.
8. After it lands: pilot flips the mode switch back to **Altitude**.

To fly again: drone on the floor, motors off. Pilot: Altitude, arm, Position. The code resets
itself after every landing.

---

## Part D: the full mission (2 m up, 2 m forward)

Only after Part C worked.

1. Log in (A2) and switch to the full version:

   ```bash
   sudo sed -i 's/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=competitor.launch.py/' /etc/skybeta/drone.env
   grep COMPETITOR /etc/skybeta/drone.env
   ```

   ✅ `COMPETITOR_PKG=snowcrashers_nav` and `COMPETITOR_LAUNCH=competitor.launch.py`.
2. Battery out, 10 seconds, battery in, so it restarts with the new setting. Clear space: 3 m
   ahead, 2.5 m overhead. Don't touch it for 1 minute.
3. Run the A7 checks. The ready line must now say `up 2.0 m, forward 2.0 m`.
4. Fly exactly as in Part C, steps 4 to 8.

To go back to the small version: run the A5 command again, then power-cycle.

---

## Results afterwards

```bash
docker exec skybeta-stack tmux capture-pane -p -S -500 -t skybeta:competitor | grep -E "RESULT|\[(CLIMB|FORWARD|DESCEND|TOUCHDOWN|DONE|ABORTED)\]"
```

---

## Something went wrong

| You see | Do this |
|---|---|
| `Permission denied` when logging in | Wrong password |
| `hostname` isn't `skybeta-03` | `exit` at once. Wrong drone |
| doctor `uart-traffic FAIL` | Battery out 10 s, in, wait 1 min, re-check. Still FAIL: Pixhawk has no power or its cable is loose: Kelvin |
| `No such container: skybeta-stack` | The software isn't running: do A6, then `host/autostart.sh status` |
| `team_ws build failed` (A7) | Something in `~/team_ws/src` doesn't build: move everything except `snowcrashers_nav` out (A3), power-cycle |
| No `PX4 link up` (A7) | Same as `uart-traffic FAIL` |
| `"local_enu": null` | Vision isn't tracking: power-cycle, keep still 1 min, more light, textured floor |
| `staying out of Offboard: no team setpoint received` or `... s old` | Our code isn't running: check A7 (`up_forward_land ready`) |
| `staying out of Offboard: no valid local position` | Vision not tracking: see the row above |
| `staying out of Offboard: in the air` | Only switch to Position while it's on the ground |
| `armed already in Position: no autonomy` | The pilot armed with the switch on Position. Disarm, switch to Altitude, arm, then Position |
| `PX4 did not enter Offboard within 2 s` | PX4 refused: check VERSION is 0.3.3 (A3), then Kelvin |
| Motors stop about 10 s after arming without flying | The takeover didn't happen (look for a `staying out of Offboard` line), or the props are off (normal in Part B) |
| `[ABORTED] Offboard ended` in the air | The pilot took over, or vision stopped: the pilot lands it |
| `Still armed ... pilot, disarm` | PX4 didn't notice the landing: the pilot disarms on the switch |
| `PX4 reports Offboard, but this node did not see it start` | After a landing: flip the mode switch to Altitude |
| It drifts sideways or wobbles in the air | Take over (throttle to middle). Usually vision: light, texture, keep still at power-up. **Don't touch PX4 parameters**: tell Kelvin |

## Updating the code later

From the repo folder on your laptop:

```bash
scp -r team_ws/src/snowcrashers_nav dronehacks@skybeta-03.local:team_ws/src/
```

Then battery out 10 s, in: it rebuilds by itself at start-up. Do Part B again before flying.

## Turning it off

To stop the drone running our code at power-up: `cd ~/skybeta && sudo host/autostart.sh disable`.
