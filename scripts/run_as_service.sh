#!/bin/bash

WS_PATH=${HOME}
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WS_PATH}/ws_socialdroids/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch robot_comm_serial serial.launch.py
