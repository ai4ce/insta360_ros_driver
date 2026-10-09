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
    ros-humble-imu-tools

RUN apt install -y \
    tmux

WORKDIR /workspace/ros2_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 0755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]


