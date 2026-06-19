#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_FILE="$HOME/.cache/atlas-app-robot.log"

exec > >(tee -a "$LOG_FILE") 2>&1
echo "===== $(date) ====="
echo "WORKSPACE=$WORKSPACE"

# ── CẤU HÌNH: chỉnh IP của PC đang chạy atlas_api ───────────────────────────
API_HOST="192.168.2.102:8080"
# ─────────────────────────────────────────────────────────────────────────────

export DISPLAY="${DISPLAY:-:0}"
export PATH="/usr/local/bin:/usr/bin:/bin:$PATH"

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

echo "[atlas_app_robot] Launching app..."
exec ros2 launch atlas_app_robot atlas_app_robot.launch.py \
    host:="$API_HOST" \
    fullscreen:=true
