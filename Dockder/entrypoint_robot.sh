#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Tạo symlink thiết bị nếu chưa có (dùng udev rules là tốt hơn)
# Xem Docker/udev_rules/README trong repo

echo "[atlas_robot] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "[atlas_robot] Starting a2_bringup..."

exec ros2 launch a2_bringup a2_bringup.launch.py
