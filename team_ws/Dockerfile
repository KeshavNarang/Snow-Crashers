# SnowCrashers team image: the skybeta base with one file replaced, the camera config
# (camera/oakd_vslam.yaml = stock + colour camera). Our ROS code is NOT in here (src/ is
# bind-mounted). Build: cd ~/skybeta && host/run.sh build, then power-cycle.
# Undo (stock camera): mv ~/team_ws/Dockerfile elsewhere, power-cycle.
ARG BASE
FROM ${BASE}

ARG CONFIG=/root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup/config/oakd_vslam.yaml
COPY camera/oakd_vslam.yaml /tmp/oakd_vslam.yaml
RUN rm -f ${CONFIG} && mv /tmp/oakd_vslam.yaml ${CONFIG}
