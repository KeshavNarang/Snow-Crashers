# SnowCrashers team image: the skybeta base with the camera config replaced (camera/oakd_vslam.yaml =
# stock + colour camera) and one line added to the stock launch (converter queue depth, below). Our ROS code is NOT in here (src/ is
# bind-mounted). Build: cd ~/skybeta && host/run.sh build, then power-cycle.
# Undo (stock camera): mv ~/team_ws/Dockerfile elsewhere, power-cycle.
ARG BASE
FROM ${BASE}

ARG CONFIG=/root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup/config/oakd_vslam.yaml
COPY camera/oakd_vslam.yaml /tmp/oakd_vslam.yaml
RUN rm -f ${CONFIG} && mv /tmp/oakd_vslam.yaml ${CONFIG}

# cuVSLAM's bgr8 -> mono8 converters: keep only the newest frame waiting (stock: RELIABLE, depth 10).
# With 10 queued, a converter that briefly falls behind leaves cuVSLAM (and stereo_node, which reads
# the converters' output) up to 10 frames late: measured on skybeta-03 2026-11-08 with goalnav
# running, odometry 200-370 ms old (35 ms + 10 x 33 ms) against 60 ms when the queue drains.
# docs/logs/2026-11-08-camera-data.md. The grep fails the build if the stock line ever changes.
ARG LAUNCH=/root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup/launch/oakd_vslam.launch.py
RUN sed -i "s/^\( *\)'encoding_desired': 'mono8',$/&\n\1'input_qos_depth': 1,/" ${LAUNCH} \
 && grep -q "'input_qos_depth': 1," ${LAUNCH}
