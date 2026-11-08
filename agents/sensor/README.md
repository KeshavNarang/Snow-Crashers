# Auxiliary sensor workstream

> Current requirement: real RGB is mandatory for pad finding; enabling the existing OAK-D colour sensor is competition-permitted, confirmed by the user. Do not fake RGB-sensor values. The candidate/rule discussion below is historical; see [current camera guidance](../../docs/CAMERAS.md).

Status (2026-11-08): leading idea = a small downward COLOUR camera so the pad detector can see the mat.
Criteria were written first: `agents/sensor/CRITERIA.md`. Candidate scoring below; nothing bought or
installed. Update this file as decisions are made.

Constraints (rule 4): < US$100 market price (not what we paid), < 50 g incl. board/lens, powered
from drone battery, no extra computer (a microcontroller reading it counts toward the sensor),
declared at inspection (part number, price link, mass). No GNSS, beacons, UWB, mocap, lasers > Class 1.
Judges score how well we *use* it in the presentation, so plan a measurable story (before/after).

Open questions: which sensor; where it mounts (CAD in `cad/`); interface (I2C/UART/USB to Jetson
vs Pixhawk); how goalnav consumes it (`goalnav/drone_ref/goalnav_node.py`); sim model in `localsim/`.
Ideas to evaluate: downward optical-flow (PX4 currently has `08_no_optical_flow.params`), side/rear
ToF rangers for wall clearance, IMU/mag, downward camera for pad detection.


## Downward colour camera: candidates scored against CRITERIA.md (2026-11-08)

Prices and weights are from retailer search snippets (some in other currencies, some "item weight" that
may include packaging), NOT verified. Every UNKNOWN must be checked before buying.

| Candidate | C1 price <$100 | C2 <50 g | C3 colour | C4/C5 Jetson+ROS | C8 FOV | C9 shutter | Verdict |
|---|---|---|---|---|---|---|---|
| **Option 0: the OAK-D S2's own colour sensor** (12 MP IMX378, already on the drone) | free | 0 g | yes | needs the driver pipeline changed in `~/skybeta/.../oakd_vslam.yaml` (organizer-owned) | wide | rolling | **Allowed and required for the pad mission.** No added hardware/mass; enable and verify real RGB. Performance/configuration work remains, not competition permission. |
| **Arducam OV9782 USB UVC colour global shutter, 1 MP 1280x800 (B0385)** | UNKNOWN: AU$96.95 incl. GST at one retailer (about US$55-60?), no US price found | UNKNOWN: ~50 g listed as item weight (may include packaging); weigh it | yes | UVC: works with stock Linux; v4l2_camera/usb_cam | UNKNOWN: depends on lens variant, pick a wide one | global | **Best candidate.** Verify US price, weigh the bare module, lens FOV |
| Arducam IMX219 wide angle, CSI, Orin Nano version (JetPack 6.2 supported) | $26.99 listed | 15 g listed (an older SKU says 8 g); board 25x24 mm | yes | CSI, supported on JetPack 6.2; cable must be the 22-to-15 pin type for Orin Nano | 160-175 deg fisheye: needs calibration | rolling (blur at speed) | **Backup.** Cheapest and lightest; needs a reachable CSI connector and a ribbon routed on the frame |
| Arducam AR0234 colour global shutter, CSI | $109.99 | UNKNOWN | yes | CSI, Orin supported | - | global | **FAIL C1** (over $100) |
| e-con IMX568 colour global shutter | UNKNOWN | 26 g listed | yes | listed for AGX Orin only | - | global | Likely over budget; not pursued |
| Arducam OV9281 / OV2311 global shutter | ~EUR 62 | ~70 g shipping | **mono** | yes | - | global | **FAIL C3** (no colour) |

### What skybeta-03 shows for C7 (read-only, 2026-11-08, stack up 1 min)
- USB3 side: a Realtek 4-port hub holding only the OAK-D (Movidius VPU); USB2 side: a 4-port hub with nothing
  attached; Bluetooth radio on its own port. So the bus has free ports.
- No `/dev/video*` devices at all: no CSI camera is in use. Whether a CSI connector or a USB-A socket is
  physically reachable on the airframe cannot be told from software: look at the drone.

### To do before buying (cheap, in this order)
1. Ask the organizers whether the OAK-D colour stream can be enabled (Option 0). Costs nothing.
2. Look at the airframe: which USB-A / CSI connectors are free and where a camera could sit looking down.
3. Check the US price and weigh the bare OV9782 module (rule 4.3 weighs the sensor as sold).
4. Read the module's USB current (rule 4.7, no extra battery) and its UVC exposure controls.
5. Write the acceptance evals first (CRITERIA.md "After purchase"), then integrate.

Sources: Arducam product pages for the USB2.0 UVC global shutter module and the IMX219 Orin kit; Core
Electronics listing for the OV9782 board (B0385); Arducam AR0234 listing via a retailer; e-con Systems IMX568 datasheet.
