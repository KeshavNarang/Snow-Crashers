# Downward camera: acceptance criteria (written BEFORE looking at any candidate)

Purpose: give the pad detector (`goalnav/drone_ref/goalnav/pad_detect.py`: chromaticity histogram of the
mat's colours, rays cast onto the floor plane) a colour image on the real drone. The OAK-D S2 has a colour
sensor but the stack's driver publishes only the mono pair (config is organizer-owned, `~/skybeta`), so
the pad is never seen on the drone and the mission flies home (agents/logs/2026-11-07-goalnav-hardware.md).
Worth: CP5 landing 7 pts + the x1.25 return bonus on run points + the aux-sensor presentation story (rule 4.6).

Each candidate is scored PASS / FAIL / UNKNOWN per line; a FAIL on a MUST rejects it. UNKNOWN on a MUST
means "verify before buying".

| # | Criterion | Level | Source |
|---|---|---|---|
| C1 | New retail price (market value, not what we paid) under US$100 with all parts it needs; target <= US$80 | MUST | rule 4.1, 4.2, 4.4 |
| C2 | Mass as sold incl. board, enclosure, lens under 50 g; target <= 35 g | MUST | rule 4.1, 4.3 |
| C3 | Colour (RGB/Bayer), >= 640x480 at >= 15 fps | MUST | pad_detect.py needs chromaticity |
| C4 | Works on the Jetson Orin Nano (JetPack 6 / Ubuntu 22.04, aarch64) with a standard driver (UVC, or a CSI sensor already supported by JetPack); no kernel module build | MUST | no time for driver work |
| C5 | A ROS 2 Humble image path exists (v4l2_camera / usb_cam / GStreamer) inside the skybeta container | MUST | stack is ROS 2 in a container |
| C6 | Powered from the drone battery (via the Jetson port), no extra battery, <= ~1 W | MUST | rule 4.7 |
| C7 | A port is actually free and reachable on the drone (USB or CSI), checked on skybeta-03 | MUST | physical |
| C8 | Horizontal FOV >= 75 deg so a ~0.8 m mat fits at 0.5 m height | SHOULD | mat size, pad_detect footprint |
| C9 | Manual exposure control (short exposure to limit blur at <= 1 m/s); global shutter preferred | SHOULD | motion blur |
| C10 | Fixed focus usable from 0.3 to 1.5 m (no autofocus hunting) | SHOULD | landing approach |
| C11 | Small enough to mount under the frame without touching props, the OAK-D, or the rangefinder's view; cable long enough | SHOULD | rule 3.6, 3.2 |
| C12 | Part number and a retail listing link we can declare at inspection | MUST | rule 4.5 |

## Decision rule
Pick the cheapest candidate with no MUST FAIL and the most SHOULD PASS. If nothing passes C7, the answer is
"no camera fits; use the OAK-D colour sensor via an organizer config request".

## After purchase (acceptance tests, to build as evals before integrating)
- A recorded image of the real mat from 0.5, 1.0 and 1.5 m must be found by `PadDetector` within 0.1 m.
- Camera + stereo_depth + cuVSLAM together: odometry stays >= 28 Hz, depth rate does not drop > 10 %.
- Camera extrinsics (mount offset/pitch) calibrated once; floor-plane error < 2 cm at 1 m.
