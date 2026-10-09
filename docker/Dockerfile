FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV PIP_ROOT_USER_ACTION=ignore
ENV ROS_DISTRO=humble

RUN apt update && apt install -y \
    ros-humble-rqt-image-view \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-imu-tools \
    ros-humble-tf2-geometry-msgs \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

RUN apt install -y \
    tmux

# Optional: Insta360 MediaSDK. Installed to /opt/MediaSDK-<version>-linux when its
# .deb is present in lib/; the bind mount keeps the 2 GB file out of the image layers.
RUN --mount=type=bind,source=lib,target=/tmp/insta360_lib \
    deb="$(ls /tmp/insta360_lib/MediaSDK-*.deb 2>/dev/null | head -n1)"; \
    if [ -n "$deb" ]; then apt update && apt install -y "$deb"; \
    else echo "No MediaSDK .deb in lib/: media_sdk_stitcher will be skipped"; fi

WORKDIR /workspace/ros2_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 0755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]


