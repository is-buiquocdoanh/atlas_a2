#!/bin/bash
# Tự tìm workspace từ vị trí file này — không cần chỉnh đường dẫn cứng
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

exec ros2 launch a2_bringup a2_bringup_robot.launch.py
