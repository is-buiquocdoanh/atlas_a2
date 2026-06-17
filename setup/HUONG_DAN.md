# Hướng dẫn Setup — Atlas A2

## Kiến trúc triển khai
![Kiến trúc](/docs/atlas_ros2_architecture.png)

> **Lưu ý quan trọng:** PC và Robot phải cùng mạng WiFi/LAN, cùng `ROS_DOMAIN_ID`.

---

## Cấu trúc thư mục setup/

```
setup/
├── install.sh           # Cài autorun (chạy 1 lần duy nhất, cần sudo)
├── start_bringup.sh     # Khởi động bringup (sensors, drivers)
├── start_app_robot.sh   # Khởi động app màn hình cảm ứng trên robot
└── HUONG_DAN.md         # File này
```

---

## Bước 1 — Cấu hình IP của PC

Mở file `setup/start_app_robot.sh`, sửa dòng `API_HOST`:

```bash
# ── CẤU HÌNH: chỉnh IP của PC đang chạy atlas_api ──
API_HOST="192.168.1.100:8080"   # <-- đổi thành IP thực của PC
```

Kiểm tra IP của PC:
```bash
# Trên PC (Linux/Mac)
ip addr show | grep "inet " | grep -v 127.0.0.1

# Trên PC (Windows)
ipconfig
```

---

## Bước 2 — Cài autorun (chạy 1 lần sau khi clone repo)

```bash
cd ~/atlas_a2
sudo bash setup/install.sh
```

Script tự động:
- Tạo systemd service `atlas-bringup` → bringup tự chạy khi boot
- Tạo autostart `~/.config/autostart/atlas-app-robot.desktop` → app robot tự mở khi đăng nhập desktop
- Tạo shortcut `~/Desktop/Atlas_A2` → click để mở app thủ công
- Thêm user vào group `dialout` và `plugdev` (cần để truy cập cổng serial/USB)

---

## Bước 3 — Kiểm tra sau cài đặt

```bash
# Xem trạng thái bringup service
sudo systemctl status atlas-bringup

# Xem log bringup realtime
journalctl -u atlas-bringup -f

# Khởi động bringup ngay (không cần reboot)
sudo systemctl start atlas-bringup

# Dừng bringup
sudo systemctl stop atlas-bringup
```

---

## Chạy thủ công (không qua systemd)

```bash
# Trên robot — chạy bringup
bash ~/atlas_a2/setup/start_bringup.sh

# Trên robot — chạy app màn hình (trong terminal khác)
bash ~/atlas_a2/setup/start_app_robot.sh
```

---

## Thứ tự khởi động đúng

```
1. PC:    ros2 launch atlas_api ...   (phải chạy trước)
2. Robot: start_bringup.sh            (khởi động sensors/motors/nav2)
3. Robot: start_app_robot.sh          (tự chờ API_HOST sẵn sàng rồi mới mở app)
```

`start_app_robot.sh` tự động chờ tối đa 30 giây cho đến khi PC's atlas_api phản hồi trước khi mở app — nên không cần lo thứ tự giữa bước 2 và 3.

---

## Khi đổi IP mạng (robot hoặc PC đổi IP)

Chỉ cần sửa 1 dòng trong `setup/start_app_robot.sh`:
```bash
API_HOST="<IP mới của PC>:8080"
```

Không cần chạy lại `install.sh`.

---

## Gỡ autorun

```bash
# Dừng và xóa systemd service
sudo systemctl stop atlas-bringup
sudo systemctl disable atlas-bringup
sudo rm /etc/systemd/system/atlas-bringup.service
sudo systemctl daemon-reload

# Xóa autostart app
rm ~/.config/autostart/atlas-app-robot.desktop
rm ~/Desktop/Atlas_A2.desktop
```

---

## Xử lý sự cố thường gặp

| Vấn đề | Kiểm tra |
|--------|----------|
| App không kết nối được PC | `curl http://<PC_IP>:8080/atlas/status` từ robot |
| Không thấy topic ROS2 từ robot | `echo $ROS_DOMAIN_ID` — phải giống nhau trên cả hai |
| rplidar crash khi bringup | Dùng powered USB hub, xem `dmesg \| grep cp210x` |
| Bringup không tự chạy khi boot | `journalctl -u atlas-bringup -n 50` để xem lý do |
| App không tự mở sau login | Kiểm tra `~/.config/autostart/atlas-app-robot.desktop` có tồn tại không |
