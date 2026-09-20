#!/usr/bin/env bash
# Convenience wrapper: launches the mock robot with ROS_LOCALHOST_ONLY=1
# Required on macOS when DDS multicast discovery isn't working over the network.
source /Users/giuliomastromartino/Documents/Polispace/LOCAL/ROS_Control/Ros_control/install/setup.zsh
export ROS_LOCALHOST_ONLY=1
ros2 launch my_robot_bringup my_robot_mock.launch.py "$@"
