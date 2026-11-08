# drone_tools: is something bottlenecking the Jetson's cores?

Two read-only monitors for skybeta-03, plus the evals that test them. Nothing here publishes, arms or changes the system.

| Tool | Where it runs | Answers |
|---|---|---|
| `cpu_watch.py` | drone **host** (needs to see all processes) | Which process or thread is using the cores? Is a core saturated, is something single-thread-bound, is the CPU over-committed, is there a stray process, throttling, heat, low memory? |
| `slam_gaps.py` | drone **host** | How many frames does cuVSLAM actually lose (effective camera rate), in what bursts, and do the bursts line up with CPU load (`--cpu-log` from `cpu_watch --log`)? It reads cuVSLAM's own "frame gap above threshold" warnings, so it measures the delivered feed from the consumer that matters. |
| `topic_rates.py` | inside `skybeta-stack` (ROS sourced) | How fast, how steady and how old are the camera, IMU and odometry topics? Low overhead (raw subscription), unlike `ros2 topic hz`, which deserializes 2 MB images in Python and distorts the rate it measures. |

```bash
# on the drone (tools/drone shell), after tools/drone push tools/drone_tools '~/team_ws/tools/'
python3 ~/team_ws/tools/drone_tools/cpu_watch.py                # live view, Ctrl-C to stop
python3 ~/team_ws/tools/drone_tools/cpu_watch.py --once 15      # one report + VERDICT
python3 ~/team_ws/tools/drone_tools/cpu_watch.py --once 15 --json
python3 ~/team_ws/tools/drone_tools/slam_gaps.py                 # real frame loss seen by cuVSLAM
# correlate: record CPU for a while, then compare with the gap times
timeout 300 python3 ~/team_ws/tools/drone_tools/cpu_watch.py --refresh 1 --log /tmp/cpu.jsonl >/dev/null
python3 ~/team_ws/tools/drone_tools/slam_gaps.py --cpu-log /tmp/cpu.jsonl
docker exec skybeta-stack bash -lc "source /opt/ros/humble/setup.bash; python3 /root/team_ws/tools/drone_tools/topic_rates.py --secs 10"
```

`cpu_watch` ends with a **VERDICT**. Empty means no bottleneck was found. Lines mean:
`CORE n SATURATED` (names who ran on it), `SINGLE-THREAD BOUND` (more cores will not help),
`CPU OVER-COMMITTED` (run queue above the core count), `STRAY / NON-STACK PROCESS` (not part of the
stack and busy), `CLOCKS BELOW MAX`, `HOT`, `MEMORY PRESSURE`.

**Do not judge the camera's frame rate from `topic_rates` or `ros2 topic hz`.** A plain Python subscriber sees
24-26 Hz on this drone while cuVSLAM receives 29.4 Hz (`slam_gaps.py`). Use `topic_rates` for message age and
relative comparisons, and `slam_gaps` for the real delivered rate.

Reading `topic_rates`: `late` counts gaps over 1.5x the median (dropped or late frames). `age` is
now minus `header.stamp` on arrival. Batched topics such as the IMU have a bimodal gap, so their
`late` count is meaningless; look at `hz`. The stereo-pair offset uses the nearest right frame, so it
cannot see a constant one-frame shift, only jitter and mismatches.

## Evals
```bash
cd tools/drone_tools && python3 -m unittest -v     # 45 tests, no drone and no ROS needed
bash eval_live.sh                                  # on the drone host: pinned self-stopping burner
```
`eval_live.sh` runs a busy loop pinned to core 5 for 14 s and requires `cpu_watch` to name that core,
that process at ~1.0 cores, call it a stray, and call it single-thread-bound. Run it after any change
to `cpu_watch.py`. The unit evals include cases where the tool must stay quiet, and bugs found on the
real drone (a sysfs read raising `TypeError`, ROS discovery being asynchronous, `dockerd` mistaken for
a container).
