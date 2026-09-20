#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS 2 underlay and workspace
source /Users/giuliomastromartino/Documents/Polispace/ros2_humble/local_setup.zsh
source "$WS_DIR/install/local_setup.zsh"

# ROS 2 config
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42

# Gazebo network config (suppresses macOS multicast errors)
export GZ_PARTITION=8
export GZ_IP=127.0.0.1

# OGRE resource paths (needed for Gazebo GUI rendering)
CONDA_PREFIX=/Users/giuliomastromartino/Documents/Polispace/ros2_humble
export OGRE_RESOURCE_PATH="$CONDA_PREFIX/lib/OGRE"
export OGRE2_RESOURCE_PATH="$CONDA_PREFIX/lib/OGRE-Next"

# LIBRARY_PATH for macOS atomic stub
export LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"

# Kill any existing gazebo processes
pkill -f "gz sim" 2>/dev/null || true
sleep 1

echo "Starting arm-only Gazebo + MoveIt demo..."

ros2 launch rover_arm_gazebo arm_gazebo_demo.launch.py 2>&1
