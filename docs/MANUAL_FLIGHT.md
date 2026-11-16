# Flying by hand, with everything recorded

For a flight where the **pilot flies the drone with the RC transmitter** and the data comes back to
your laptop afterwards. Run the commands in **Git Bash** on your laptop, from the repo folder.

**Why there's a separate mode.** Our normal missions (overwall_nav, goalnav) take control of the drone
as soon as the pilot arms and switches to Position. The manual mode runs **only a recorder**: nothing
on the drone can take control away from the pilot. Position mode still works as usual, because the
organizers' camera tracking (cuVSLAM) keeps running.

## Before you go to the pad (once)

- Laptop on the same Wi-Fi as the drone. Have the drone password ready; each command asks for it once.
- `git checkout main && git pull`
- Optional, for a quick summary after each flight: `pip install rosbags`

## At the pad

| Step | Do this | You should see |
|---|---|---|
| 1 | `tools/manual-flight setup "<your name>"` | `NOW: power-cycle the drone` |
| 2 | **Power-cycle:** battery out 10 s, back in, wait **3 minutes** | |
| 3 | `tools/manual-flight check` | every line `OK`, then **`GO: arm and fly by hand.`** |
| 4 | **Pilot flies.** Kill switch in hand, as always. | |
| 5 | Land and disarm. **Leave the battery in.** | |
| 6 | `tools/manual-flight pull` | `complete: flight-logs/<date-time>` |
| 7 | *(optional)* PX4's own log: USB cable to the **Pixhawk**, then QGroundControl → **Analyze Tools → Log Download** → newest → Download | a `.ulg` file |

**Another flight:** swap the battery (a power-cycle starts a new recording folder), wait 3 minutes,
then go back to step 3.

**Finished for the day:** `tools/manual-flight restore`, then power-cycle. This puts back the
mission the drone ran before step 1 and frees the drone for teammates.

## What gets recorded

| | What | Where |
|---|---|---|
| **Inputs** | the pilot's sticks and switches, as PX4 received them | `flight-logs/<date-time>/bag` and the PX4 log |
| **Outputs** | what the drone did: position, attitude, speed, thrust and torque commands, flight mode, armed or landed, battery | same |
| **Sensing** | the cameras (mono pair and colour), IMU, cuVSLAM's position, what was fed to PX4 | `bag` |
| **Logs** | every program's log | `flight-logs/<date-time>/ros` |

**Motor outputs and full-rate sticks** are only in the PX4 log (step 7). Download it every time,
because the drone's copy of the stick topic hasn't been checked on this drone yet.

**Opening the files:** the bag (`.mcap`) opens in **Foxglove** (free app: drag the `bag` folder in).
The PX4 log (`.ulg`) opens at **review.px4.io** or in **PlotJuggler**.

## If something's wrong

| You see | Do |
|---|---|
| `FAIL mission is '...'` | You skipped step 1 or 2. Run `setup` again, then power-cycle. |
| `FAIL something that can fly the drone is running` | **Don't fly.** Power-cycle. Still there? Run `setup` again. |
| `FAIL recorder not running` / `log ... not growing` | Power-cycle, wait the full 3 minutes, `check` again. |
| `FAIL the stack container ... is not running` | The drone hasn't finished starting. Wait 1–2 minutes, `check` again. |
| `WARN only N GB free` | Video uses about 3 GB per minute. `tools/manual-flight setup "<name>" --no-video`, power-cycle (everything except camera images). |
| `pull` is slow | Video is big. A USB-C cable to the **Jetson** is faster: put `DRONE_HOST=192.168.55.1` in front of the command. |
| Battery came out before `pull` | The data is still on the drone, but the last bag wasn't closed. Power on, wait 3 minutes, then `tools/fly logs list` and `tools/fly logs <folder>` to copy it. |

Everything here only reads from the drone or selects which program it runs at power-up. It never
changes PX4 parameters.
