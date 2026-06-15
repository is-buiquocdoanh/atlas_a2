#!/bin/bash
# Tự tìm workspace từ vị trí file này — không cần chỉnh đường dẫn cứng
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Đợi bringup service sẵn sàng (atlas_api lắng nghe port 8080)
echo "[atlas_app_robot] Chờ API robot sẵn sàng..."
for i in $(seq 1 30); do
    if curl -sf http://localhost:8080/atlas/status > /dev/null 2>&1; then
        echo "[atlas_app_robot] API sẵn sàng — khởi động app"
        break
    fi
    sleep 1
done

exec ros2 launch atlas_app_robot atlas_app_robot.launch.py \
    host:=localhost:8080 \
    fullscreen:=true
