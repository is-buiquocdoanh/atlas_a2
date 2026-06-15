#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

echo "[atlas_app] ROBOT_HOST=${ROBOT_HOST:-localhost:8080}"
echo "[atlas_app] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

exec ros2 launch atlas_app atlas_app.launch.py
