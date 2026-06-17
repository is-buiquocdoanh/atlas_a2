#!/bin/bash
# ============================================================
# Atlas A2 — Setup autorun trên robot
# Chạy một lần sau khi clone repo:
#   cd atlas_a2 && sudo bash setup/install.sh
# Hoạt động với mọi username và tên thư mục workspace.
# ============================================================
set -e

# ── Tự phát hiện workspace và user ───────────────────────────────────────────
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# $SUDO_USER là user thực khi chạy sudo (không phải root)
USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Cần chạy với sudo: sudo bash setup/install.sh"
    exit 1
fi

echo "======================================================"
echo " Atlas A2 — Cài đặt autorun"
echo " Workspace : $WORKSPACE"
echo " User      : $USER_NAME ($USER_HOME)"
echo "======================================================"

# ── 1. Đặt quyền execute cho scripts ─────────────────────────────────────────
echo "[1/5] Đặt quyền execute..."
chmod +x "$WORKSPACE/setup/start_bringup.sh"
chmod +x "$WORKSPACE/setup/start_app_robot.sh"

# ── 2. Tạo và cài systemd service ────────────────────────────────────────────
echo "[2/5] Tạo systemd service (atlas-bringup)..."
cat > /etc/systemd/system/atlas-bringup.service << EOF
[Unit]
Description=Atlas A2 — Robot Bringup (sensors, drivers, API, YOLO)
After=network.target systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
Type=simple
User=$USER_NAME
Group=$USER_NAME
Environment="HOME=$USER_HOME"
Environment="ROS_DOMAIN_ID=4"
ExecStartPre=/bin/sleep 15
ExecStart=$WORKSPACE/setup/start_bringup.sh
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=atlas-bringup

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable atlas-bringup.service
echo "      OK — sẽ tự chạy khi boot"

# ── 3. Tạo và cài XDG autostart ──────────────────────────────────────────────
echo "[3/5] Tạo autostart app robot..."
AUTOSTART_DIR="$USER_HOME/.config/autostart"
mkdir -p "$AUTOSTART_DIR"
cat > "$AUTOSTART_DIR/atlas-app-robot.desktop" << EOF
[Desktop Entry]
Type=Application
Name=Atlas A2 Robot App
Exec=$WORKSPACE/setup/start_app_robot.sh
Terminal=false
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=8
EOF
chown "$USER_NAME:$USER_NAME" "$AUTOSTART_DIR/atlas-app-robot.desktop"
echo "      OK — app sẽ tự khởi động khi đăng nhập desktop"

# ── 4. Tạo icon trên Desktop ─────────────────────────────────────────────────
echo "[4/5] Tạo shortcut trên Desktop..."
DESKTOP_DIR="$USER_HOME/Desktop"
mkdir -p "$DESKTOP_DIR"
cat > "$DESKTOP_DIR/Atlas_A2.desktop" << EOF
[Desktop Entry]
Type=Application
Name=Atlas A2
Comment=Mở ứng dụng điều khiển Atlas A2
Exec=$WORKSPACE/setup/start_app_robot.sh
Terminal=false
StartupNotify=true
EOF
chown "$USER_NAME:$USER_NAME" "$DESKTOP_DIR/Atlas_A2.desktop"
chmod +x "$DESKTOP_DIR/Atlas_A2.desktop"
echo "      OK — icon tại ~/Desktop/Atlas_A2"

# ── 5. Group permissions ──────────────────────────────────────────────────────
echo "[5/5] Kiểm tra group permissions..."
for grp in dialout plugdev; do
    if ! groups "$USER_NAME" | grep -q "$grp"; then
        usermod -aG "$grp" "$USER_NAME"
        echo "      Đã thêm $USER_NAME vào group $grp"
    fi
done

echo ""
echo "======================================================"
echo " Hoàn tất!"
echo ""
echo " Khởi động bringup ngay (không cần reboot):"
echo "   sudo systemctl start atlas-bringup"
echo ""
echo " Xem log:"
echo "   journalctl -u atlas-bringup -f"
echo ""
echo " Reboot để test autorun đầy đủ:"
echo "   sudo reboot"
echo "======================================================"
