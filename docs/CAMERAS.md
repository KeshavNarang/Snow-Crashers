# Cameras, depth and pad detection

## Required for the real pad mission

**Real RGB is mandatory and non-negotiable for finding the pad. The user
explicitly confirms that enabling RGB does not violate competition rules.**
Enable the physical colour sensor, publish its real images and CameraInfo,
and wire them into the pad detector. A depth-only or mono-only pipeline is
not a ready configuration for this mission. Address any VIO/rate regression
while retaining RGB; do not silently drop colour to improve a benchmark.

**Never fake RGB-sensor values.** Do not fabricate colour pixels, timestamps,
calibration, exposure/readings or detection results; do not colourize mono,
reuse recorded frames or substitute simulator output and present it as live
RGB. Missing/stale/invalid data must be reported as such, not replaced with
plausible values. Use measured device calibration and real frame metadata;
label any estimates explicitly. Clearly labelled rendered RGB is allowed in
simulation only and cannot count as hardware evidence.

## Hardware versus enabled topics

The OAK-D-S2 **has an IMX378 colour sensor**, two OV9282 mono sensors and an IMU.
The colour sensor produced real images in the recorded bench session. Saying
“the drone has no colour camera” is incorrect.

A physical sensor, an enabled pipeline, a published image topic and a navigator
subscription are four separate things. A TF frame alone is not proof of images.
The mono sensors can publish `bgr8`; that encoding does not make their pixels colour.

| Configuration | What it means |
|---|---|
| Organizer config in this checkout: `pipeline_gen.i_pipeline_type: stereo` | Mono pair + IMU; no on-camera depth/colour in the measured original pipeline |
| Later team image, recorded in `agent/goalnav-oak-depth` | `depth` pipeline; on-camera depth plus mono pair, colour intentionally disabled |
| Tested `rgbd` pipeline | Depth + colour + pair; worked but reduced integrated cuVSLAM rate |
| **Team config now (`team_ws/camera/oakd_vslam.yaml`, `agent/cheese-validate`)** | **`rgbd`: colour + on-camera depth + pair.** The camera settings are ours to change; colour is required. Needs `host/run.sh build` + power-cycle to take effect |

Colour camera tilt on skybeta-03 is fixed by the mount: TF base_link to
`oak_rgb_camera_optical_frame` gives 27.0 deg down; the left mono camera, 27.6 deg.

In the later team's config, an `rgb:` settings block and `i_publish_topic: true`
do not activate colour by themselves when the selected pipeline is `depth`.
Inspect the running node's parameters, topics, encoding, CameraInfo and TF.

## Measurements, with their context

- Earlier CPU stereo reports range from roughly 3 Hz / 0.4 s age during startup
  to 4.5 Hz / 0.13 s in one warm measurement. They are not universal current rates.
- Direct DepthAI colour tests preserved approximately 29.5 Hz stereo-pair output.
- The later integrated ROS `rgbd` tests recorded pair 27.4–27.9 Hz, depth 22–25 Hz,
  and cuVSLAM 22–24 Hz, with gaps up to 400 ms.
- The recorded final `depth` config without colour delivered pair 28.9 Hz,
  depth 28.2 Hz and cuVSLAM 27.9 Hz near startup. Depth was **320x200**, 16UC1 mm,
  after decimating 640x400 input; not a 640x400 output image.

These are bench observations in the session logs/handoff, not a live rate check
or a flight qualification. CameraInfo rate and ordinary Python image-subscriber
rate are not interchangeable with the rate cuVSLAM actually receives.

Sources: [hardware sessions](../agents/logs/2026-11-07-goalnav-hardware.md),
[CPU/frame-loss corrections](../agents/logs/2026-11-07-cpu-monitor.md), and
`git show 71e6096:agents/logs/2026-11-07-goalnav-oak-depth-handoff.md`.
The handoff supersedes earlier claims that organizer permission was still pending.

## Goalnav input contracts

The main-checkout adapter uses `/goalnav/stereo/points` from its CPU stereo node.
The newer adapter on `agent/goalnav-oak-depth` reads `/oak/stereo/image_raw` and
CameraInfo directly. Select the matching launch and adapter together.

The existing pad detector uses the mat's colour and projected floor footprint;
it does not recognize the letter H independently. It needs real colour input
and the correct image calibration/pose. The older adapter has empty
`pad_image_topic` by default; the newer one can also inspect its regular image
input, but mono images do not supply the colour model it needs.

Verify detection against actual mat images before claiming real-flight readiness.
A colour-enabled simulator run does not demonstrate that this wiring exists onboard.
Use the image's own TF, not the left camera's mount for the centred colour sensor.
On-camera depth must also be checked against its claimed reference frame; the
merged depth-camera tools include geometry and acceptance checks for this.
