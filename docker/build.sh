#!/bin/bash

docker build docker/ -t x64_foxy_melodic_bridge

# SOMEHOW ONLY WORKS WITH INSTANCES ON ON UBUNTU (20.04), OWERWISE SPAMMING .SO VERSIONS MISMATHCES

# ros noetic installation http://wiki.ros.org/noetic/Installation/Ubuntu (without source /opt/ros/noetic/setup.bash)
# source ${ROS1_INSTALL_PATH}/setup.bash
# source ${ROS2_INSTALL_PATH}/setup.bash
# git clone -b foxy https://github.com/ros2/ros1_bridge.git
# colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
# source install/setup.bash
# ros2 run ros1_bridge dynamic_bridge