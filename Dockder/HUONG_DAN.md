# Hướng dẫn Docker — Atlas A2

## Tổng quan kiến trúc

```
┌──────────────────────────────────┐     WiFi / LAN
│           ROBOT                  │ ◄──────────────── │  PC / Laptop  │
│                                  │                    │               │
│  docker-compose.robot.yml        │                    │  docker-compose.app.yml
│  ┌────────────────────────────┐  │                    │  ┌──────────────────┐
│  │   atlas_a2:robot           │  │   HTTP :8080       │  │  atlas_a2:app    │
│  │                            │  │ ◄──────────────    │  │                  │
│  │  a2_bringup   (sensors)    │  │   ROS2 DDS topics  │  │  atlas_app       │
│  │  atlas_api    (:8080)      │  │ ◄──────────────    │  │  (PyQt5 GUI)     │
│  │  nav2 / SLAM               │  │                    │  └──────────────────┘
│  │  yolov8       (:6060)      │  │
│  └────────────────────────────┘  │
│                                  │
│  /dev/usbcan   ← motor driver    │
│  /dev/rplidar  ← LiDAR           │
│  /dev/video2   ← camera          │
│  /dev/battery  ← BMS             │
│  /dev/esp32    ← collision       │
│  /dev/magnetic ← line sensor     │
└──────────────────────────────────┘
```

---

## Cấu trúc thư mục Docker

```
Dockder/
├── Dockerfile.robot          # Image cho robot (bringup + nav2 + API + YOLO)
├── Dockerfile.app            # Image cho PC (chỉ app điều khiển PyQt5)
├── docker-compose.robot.yml  # Deploy lên robot
├── docker-compose.app.yml    # Deploy lên PC
├── .env                      # Biến môi trường (port USB, IP, domain ID)
├── entrypoint_robot.sh       # Script khởi động tự động khi container start
├── entrypoint_app.sh         # Script khởi động app
├── 99-atlas-a2.rules         # Udev rules — đặt tên cố định cho cổng USB
└── HUONG_DAN.md              # File này
```

---

## Giải thích từng file

### `Dockerfile.robot`
Build image đầy đủ để chạy trên robot. Bao gồm:
- Base image: `ros:humble-ros-base` (Ubuntu 22.04 + ROS2 Humble)
- Cài apt: toàn bộ ROS2 packages (nav2, slam_toolbox, rplidar, v4l2_camera, web_video_server...)
- Cài pip: `ultralytics` (YOLOv8), `flask`, `websockets`, `pyserial`, `python-can`
- Copy toàn bộ `src/` vào image và build bằng `colcon`
- Khai báo volume `/workspace/src/atlas_base/atlas_maps` để maps không bị mất khi restart

### `Dockerfile.app`
Build image nhẹ hơn chỉ để chạy app điều khiển trên PC. Bao gồm:
- Base image: `ros:humble-ros-base`
- Cài apt: `python3-pyqt5`, thư viện hiển thị X11, các ROS2 message packages
- Cài pip: `numpy`, `requests`
- Chỉ build package `atlas_app` và `atlas_slam` (cần để đọc config đường dẫn map)

### `docker-compose.robot.yml`
File deploy cho robot:
- `network_mode: host` — bắt buộc để ROS2 DDS giao tiếp qua mạng với PC
- `privileged: true` — cho phép container truy cập thiết bị phần cứng
- `devices` — mount từng cổng USB vào container
- `volumes` — lưu maps và config ra ngoài container (persistent)

### `docker-compose.app.yml`
File deploy cho PC:
- `network_mode: host` — cần để subscribe ROS2 topic từ robot qua mạng
- `volumes: /tmp/.X11-unix` — cho phép app hiển thị cửa sổ GUI trên màn hình PC
- Biến `DISPLAY`, `QT_X11_NO_MITSHM` — cấu hình X11 để PyQt5 chạy được

### `.env`
File cấu hình chung, được cả hai docker-compose đọc. **Phải chỉnh file này trước khi deploy.**

| Biến | Mô tả | Ví dụ |
|------|--------|-------|
| `ROS_DOMAIN_ID` | Domain ID của ROS2 — robot và PC phải giống nhau | `0` |
| `DEV_USBCAN` | Cổng USB-CAN (motor driver) | `/dev/usbcan` |
| `DEV_RPLIDAR` | Cổng LiDAR RPLidar A2 | `/dev/rplidar` |
| `DEV_CAMERA` | Camera USB (v4l2) | `/dev/video2` |
| `DEV_BATTERY` | Cổng BMS pin | `/dev/battery` |
| `DEV_ESP32` | Cổng ESP32 (cảm biến va chạm) | `/dev/esp32` |
| `DEV_MAGNETIC` | Cổng cảm biến từ (line follow) | `/dev/magnetic` |
| `ROBOT_HOST` | IP và port của robot API (dùng cho PC app) | `192.168.1.100:8080` |
| `DISPLAY` | Màn hình hiển thị X11 trên PC | `:0` |

### `99-atlas-a2.rules`
Udev rules giúp các cổng USB luôn có **tên cố định** (symlink) dù cắm vào port nào.

Ví dụ: thay vì `/dev/ttyUSB0` có thể thay đổi khi cắm lại → dùng `/dev/usbcan` luôn ổn định.

**Cần cài trên hệ thống robot (ngoài Docker)**, không phải trong container.

### `entrypoint_robot.sh`
Script tự động chạy khi container robot start:
```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch a2_bringup a2_bringup.launch.py
```
Lệnh này khởi động toàn bộ: sensors, motor driver, camera, YOLO, web_video_server, atlas_api.

Nav2 và SLAM được điều khiển qua **atlas_api** (HTTP) từ app, không chạy cùng lúc ngay từ đầu.

### `entrypoint_app.sh`
Script tự động chạy khi container app start:
```bash
ros2 launch atlas_app atlas_app.launch.py
```

---

## Hướng dẫn triển khai

### Bước 1 — Cài udev rules trên robot (một lần duy nhất)

```bash
# Tìm idVendor/idProduct của từng thiết bị
lsusb
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"

# Chỉnh file 99-atlas-a2.rules cho đúng với thiết bị thực tế
sudo cp Dockder/99-atlas-a2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Kiểm tra symlink đã tạo chưa
ls -la /dev/usbcan /dev/rplidar /dev/battery /dev/esp32 /dev/magnetic
```

### Bước 2 — Cấu hình `.env`

```bash
# Trên robot: kiểm tra đúng tên device
ls /dev/usbcan /dev/rplidar /dev/video2 ...

# Trên PC: đặt IP đúng của robot
nano Dockder/.env
# Sửa: ROBOT_HOST=192.168.x.x:8080
```

### Bước 3 — Build image

```bash
cd /path/to/atlas_a2

# Build image robot (chạy trên robot)
docker compose -f Dockder/docker-compose.robot.yml build

# Build image app (chạy trên PC)
docker compose -f Dockder/docker-compose.app.yml build
```

> Build lần đầu mất ~15–30 phút do cài ROS2 packages và compile workspace.
> Lần sau chỉ rebuild khi thay đổi code (`--build`).

### Bước 4 — Chạy robot

```bash
# Trên robot
docker compose -f Dockder/docker-compose.robot.yml up

# Hoặc chạy nền (background)
docker compose -f Dockder/docker-compose.robot.yml up -d

# Xem log
docker compose -f Dockder/docker-compose.robot.yml logs -f
```

### Bước 5 — Chạy app trên PC

```bash
# Cho phép Docker hiển thị GUI (chỉ cần chạy một lần mỗi session)
xhost +local:docker

# Chạy app
docker compose -f Dockder/docker-compose.app.yml up
```

---

## Lưu map

Maps được lưu vào Docker volume `atlas_maps` trên robot:
- Không mất khi restart container
- Lưu trữ tại: `/var/lib/docker/volumes/dockder_atlas_maps/_data/`

**Backup maps ra ngoài:**
```bash
docker cp atlas_robot:/workspace/src/atlas_base/atlas_maps ./maps_backup
```

**Khôi phục maps:**
```bash
docker cp ./maps_backup/. atlas_robot:/workspace/src/atlas_base/atlas_maps/
```

---

## Chạy lệnh thủ công trong container

```bash
# Mở terminal trong container đang chạy
docker exec -it atlas_robot bash

# Ví dụ: kiểm tra các topic ROS2
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
ros2 topic list

# Lưu map thủ công
ros2 run nav2_map_server map_saver_cli -f /workspace/src/atlas_base/atlas_maps/mymap
```

---

## Chuyển robot sang máy khác

```bash
# Lưu image thành file (trên robot cũ)
docker save atlas_a2:robot | gzip > atlas_a2_robot.tar.gz

# Copy sang robot mới (qua USB hoặc SCP)
scp atlas_a2_robot.tar.gz user@new_robot:/home/user/

# Load image trên robot mới
docker load < atlas_a2_robot.tar.gz

# Cài udev rules và chạy như Bước 1–4
```

---

## Xử lý sự cố thường gặp

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| `/dev/usbcan not found` | Udev rules chưa cài hoặc sai idVendor | Kiểm tra `lsusb` và cập nhật `99-atlas-a2.rules` |
| App không hiển thị cửa sổ | Chưa cho phép X11 | Chạy `xhost +local:docker` trước |
| Robot và PC không thấy topic nhau | ROS_DOMAIN_ID khác nhau | Đảm bảo `.env` có cùng `ROS_DOMAIN_ID` |
| Maps bị mất sau update image | Build lại xóa volume | Dùng `docker compose up` (không dùng `--build` nếu không cần) |
| Camera không nhận | `/dev/video2` sai | Kiểm tra `ls /dev/video*` và sửa `DEV_CAMERA` trong `.env` |
