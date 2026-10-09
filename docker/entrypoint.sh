#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /workspace/ros2_ws/install/setup.bash

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update --rosdistro "${ROS_DISTRO}"
apt-get update
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -r -y

colcon build --symlink-install && source install/setup.bash


if ! grep -qxF "#Entrypoint Setup" ~/.bashrc; then
    cat <<'EOF' >> ~/.bashrc

#Entrypoint Setup
export ROS_DOMAIN_ID=0
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_DISTRO="humble"
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ZENOH_ROUTER_CONFIG_URI="/workspace/ros2_ws/src/insta360_ros_driver/docker/RMW_ZENOH_ROUTER_CONFIG.json5"
EOF
fi

echo "================================"
echo " Insta360 Docker Container Ready"
echo " Run 'docker exec -it insta360_ros_driver bash'"
echo "================================"

exec bash
