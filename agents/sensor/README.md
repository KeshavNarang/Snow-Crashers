# Auxiliary sensor workstream

Status: sensor not yet chosen. Update this file as decisions are made.

Constraints (rule 4): < US$100 market price (not what we paid), < 50 g incl. board/lens, powered
from drone battery, no extra computer (a microcontroller reading it counts toward the sensor),
declared at inspection (part number, price link, mass). No GNSS, beacons, UWB, mocap, lasers > Class 1.
Judges score how well we *use* it in the presentation, so plan a measurable story (before/after).

Open questions: which sensor; where it mounts (CAD in `cad/`); interface (I2C/UART/USB to Jetson
vs Pixhawk); how goalnav consumes it (`goalnav/drone_ref/goalnav_node.py`); sim model in `localsim/`.
Ideas to evaluate: downward optical-flow (PX4 currently has `08_no_optical_flow.params`), side/rear
ToF rangers for wall clearance, IMU/mag, downward camera for pad detection.
