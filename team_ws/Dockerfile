# Team image (host/run.sh build): the skybeta base plus our camera config.
# The stack's camera launch reads oakd_vslam_bringup's config/oakd_vslam.yaml and takes
# no override, so we replace that file in the image: same mono pair and IMU for cuVSLAM,
# plus on-camera stereo depth and the colour camera (camera/oakd_vslam.yaml).
# Undo: rm ~/team_ws/Dockerfile (or docker rmi skybeta-team:<tag>), docker rm -f skybeta-stack.
ARG BASE
FROM ${BASE}

ARG CONFIG=/root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup/config/oakd_vslam.yaml
COPY camera/oakd_vslam.yaml /tmp/oakd_vslam.yaml
RUN rm -f ${CONFIG} && mv /tmp/oakd_vslam.yaml ${CONFIG}
