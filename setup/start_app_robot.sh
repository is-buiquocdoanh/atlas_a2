#!/bin/bash
# Tự tìm workspace từ vị trí file này — không cần chỉnh đường dẫn cứng
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ── CẤU HÌNH: chỉnh IP của PC đang chạy atlas_api ───────────────────────────
API_HOST="192.168.2.102:8080"
# ─────────────────────────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Đợi atlas_api trên PC sẵn sàng
echo "[atlas_app_robot] Chờ atlas_api ($API_HOST) sẵn sàng..."
for i in $(seq 1 30); do
    if curl -sf "http://$API_HOST/atlas/status" > /dev/null 2>&1; then
        echo "[atlas_app_robot] API sẵn sàng — khởi động app"
        break
    fi
    sleep 1
done

exec ros2 launch atlas_app_robot atlas_app_robot.launch.py \
    host:="$API_HOST" \
    fullscreen:=true
