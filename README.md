# Atlas A2 — Robot Navigation System

Hệ thống điều hướng tự động cho robot Atlas A2 dựa trên **ROS2 Humble**. Hỗ trợ SLAM, Nav2, line-follow về trạm sạc, điều khiển từ PC và màn hình cảm ứng trên robot.

---

## Kiến trúc hệ thống

```
┌─────────────────────────────────┐        WiFi (cùng subnet)
│              PC                 │◄──────────────────────────►│
│                                 │                             │
│  atlas_api  ──  REST :8080      │                             │
│  atlas_app  ──  PyQt5 UI        │                             │
│  (subscribe ROS2 DDS trực tiếp) │                             │
└─────────────────────────────────┘                             │
                                                                │
                                              ┌─────────────────────────────────┐
                                              │         Robot (Jetson Orin)     │
                                              │                                 │
                                              │  a2_bringup ── start sensors    │
                                              │  a2_driver  ── motor/USB-CAN    │
                                              │  a2_platform── lidar/mag/bat    │
                                              │  atlas_slam ── Nav2 + SLAM      │
                                              │  atlas_app_robot ── touchscreen │
                                              └─────────────────────────────────┘
```

> **ROS_DOMAIN_ID** phải giống nhau trên cả PC và robot.
> AP client isolation phải **tắt** để DDS hoạt động cross-machine.

---

## Cấu trúc thư mục

```
atlas_a2/
├── src/
│   ├── a2_bringup/          Launch files tổng hợp + line_follow + YOLO
│   ├── a2_driver/           Driver motor qua USB-CAN (TSDA-C12D)
│   ├── a2_platform/         Cảm biến phần cứng: lidar, pin, va chạm, từ trường
│   ├── atlas_base/
│   │   ├── atlas_api/       REST + WebSocket API server (Flask, chạy trên PC)
│   │   ├── atlas_app/       Ứng dụng điều khiển PyQt5 (PC, operator)
│   │   ├── atlas_app_robot/ Ứng dụng cảm ứng PyQt5 (robot, Jetson)
│   │   ├── atlas_slam/      Launch + config cho SLAM toolbox, Nav2, AMCL
│   │   ├── atlas_maps/      Bản đồ đã lưu (.yaml + .pgm)
│   │   └── atlas_web/       Web dashboard (port 8888)
│   ├── rf2o_laser_odometry/ Laser odometry từ LiDAR scan
│   ├── rplidar_ros/         Driver RPlidar A2M12
│   └── yolov8_msgs/         Message types cho YOLO object detection
└── setup/
    ├── install.sh           Cài đặt autorun systemd + desktop shortcut
    ├── start_bringup.sh     Khởi động bringup (robot)
    ├── start_app_robot.sh   Khởi động app cảm ứng (robot) — chỉnh API_HOST tại đây
    └── HUONG_DAN.md         Hướng dẫn cài đặt chi tiết
```

---

## Phần cứng

| Thành phần | Model | Kết nối |
|---|---|---|
| Main computer | Jetson Orin Nano Developer Kit (Super) | — |
| Motor driver | TSDA-C12D | USB-CAN (`/dev/usbcan`) |
| Motor | TODE brushless | CAN bus |
| Bánh xe | Supo mecanum | — |
| LiDAR | RPlidar A2M12 | USB (`/dev/rplidar`) |
| Cảm biến từ trường | 16-kênh analog | USB (`/dev/magnetic`) |
| Cảm biến va chạm | ESP32 | USB (`/dev/esp32`) |
| Pin (BMS) | — | USB (`/dev/battery`, Modbus RTU) |
| Camera | USB UVC (v4l2) | `/dev/video0` |

> **USB Hub**: Dùng hub **có nguồn riêng (powered)** để tránh lỗi ETIMEDOUT khi boot đồng thời nhiều thiết bị.

---

## Phần mềm

| Yêu cầu | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| Python | 3.10+ |
| PyQt5 | ≥ 5.15 |
| Flask | ≥ 3.0 |
| Nav2 | humble |
| slam_toolbox | humble |

---

## Cài đặt nhanh

### 1. Clone và build

```bash
git clone <repo-url> ~/atlas_a2
cd ~/atlas_a2
colcon build
source install/setup.bash
```

### 2. Cài autorun trên robot (Jetson)

```bash
sudo bash setup/install.sh
```

Script sẽ:
- Tạo systemd service `atlas-bringup` (khởi động tự động, chờ 15s cho USB ổn định)
- Tạo XDG autostart cho app cảm ứng
- Tạo icon trên Desktop
- Thêm user vào group `dialout`, `plugdev`

### 3. Cấu hình IP cho app cảm ứng

Sửa dòng `API_HOST` trong `setup/start_app_robot.sh`:

```bash
API_HOST="192.168.x.xxx:8080"   # IP của PC đang chạy atlas_api
```

### 4. Chạy thủ công

**Trên robot:**
```bash
# Terminal 1 — bringup (sensors + driver + nav2)
bash setup/start_bringup.sh

# Terminal 2 — app cảm ứng
bash setup/start_app_robot.sh
```

**Trên PC:**
```bash
# Terminal 1 — API server + ROS2 bridge
ros2 launch atlas_api atlas_api_real.launch.py

# Terminal 2 — ứng dụng điều khiển
ros2 launch atlas_app atlas_app.launch.py
```

---

## Các package chi tiết

### `a2_driver` — Driver motor

- Giao tiếp USB-CAN với TSDA-C12D (50 Hz)
- Subscribe `/cmd_vel` → gửi lệnh tốc độ 2 motor trái/phải
- Publish `/atlas/odom` (odometry từ encoder)

```
/dev/usbcan  →  driver_node.py  →  /atlas/odom
                                ←  /cmd_vel
```

### `a2_platform` — Cảm biến phần cứng

| Node | Cổng | Topic |
|---|---|---|
| `battery_node.py` | `/dev/battery` | `/atlas/battery` (20s/lần) |
| `mag_sensor_node.py` | `/dev/magnetic` | `/sensor/analog16` |
| `collision_detect_node.py` | `/dev/esp32` | `/atlas/emergency_stop` |
| `rplidar_node` | `/dev/rplidar` | `/scan` |

### `a2_bringup` — Launch tổng hợp

| File | Mục đích |
|---|---|
| `a2_bringup_robot.launch.py` | Bringup đầy đủ trên robot |
| `a2_bringup_pc.launch.py` | Bringup phía PC |
| `line_follow.py` | Bám vạch từ về trạm sạc |
| `joystick.launch.py` | Điều khiển tay bằng gamepad |

### `atlas_slam` — SLAM & Navigation

- SLAM: `slam_toolbox` (mapping) hoặc `AMCL` (localization)
- Navigation: `Nav2` (DWB controller hoặc MPPI)
- Config: `atlas_nav2_dwb.yaml`, `atlas_nav2_mppi.yaml`, `atlas_localization.yaml`

### `atlas_api` — REST API (chạy trên PC)

Base URL: `http://<PC_IP>:8080`

| Endpoint | Method | Chức năng |
|---|---|---|
| `/atlas/status` | GET | Trạng thái tổng hợp (nav, battery, docked...) |
| `/atlas/nav/goal` | POST | Gửi nav goal `{x, y, yaw}` |
| `/atlas/nav/cancel` | POST | Huỷ nav |
| `/atlas/nav/dock` | POST | Bắt đầu line-follow docking |
| `/atlas/nav/dock_stop` | POST | Dừng docking |
| `/atlas/nav/charge` | POST | Chuỗi đầy đủ: nav → dock |
| `/atlas/waypoints` | GET/POST | Danh sách waypoints |
| `/atlas/launch/status` | GET | Trạng thái các node ROS2 |

WebSocket: `ws://<PC_IP>:8081` — broadcast status 5Hz.

### `atlas_app` — Ứng dụng điều khiển (PC)

Xem [src/atlas_base/atlas_app/README.md](src/atlas_base/atlas_app/README.md)

### `atlas_app_robot` — App cảm ứng (Jetson)

- Kết nối HTTP tới `atlas_api` trên PC
- Hiển thị danh sách waypoints dạng nút lớn (touchscreen)
- Theo dõi route đang chạy, nút xác nhận cho route type `confirm`
- Fullscreen trên màn hình cảm ứng của robot

---

## Topics ROS2 chính

| Topic | Type | Nguồn |
|---|---|---|
| `/scan` | `LaserScan` | rplidar_node |
| `/atlas/scan_filtered` | `LaserScan` | relay từ `/scan` |
| `/atlas/odom` | `Odometry` | driver_node / rf2o |
| `/atlas/battery` | `BatteryState` | battery_node |
| `/atlas/emergency_stop` | `Bool` | collision_detect |
| `/atlas/imu` | `Imu` | driver_node |
| `/atlas/docked` | `Bool` | atlas_api_node |
| `/sensor/analog16` | `UInt16MultiArray` | mag_sensor_node |
| `/cmd_vel` | `Twist` | atlas_api / atlas_app |
| `/cmd_vel_mag` | `Twist` | line_follow (twist_mux) |
| `/map` | `OccupancyGrid` | slam_toolbox / map_server |
| `/plan` | `Path` | nav2 planner |

---

## Hệ thống tự về trạm sạc (Docking)

```
atlas_api nhận lệnh dock
    │
    ▼
Spawn subprocess: line_follow.py
    │
    ├─ INIT → tìm vạch từ
    ├─ SEARCH → quét ±45° nếu không thấy vạch ngay
    ├─ FOLLOW → bám vạch từ về trạm sạc
    └─ STOPPED → hết vạch = ở trạm sạc
         │
         ├─ 5s publish zero velocity
         └─ exit(0) ── atlas_api_node phát hiện ──► /atlas/docked = true
```

**Undock tự động:** Khi có nav goal mới mà robot đang ở trạm sạc:
1. Dừng docking
2. Tiến thẳng 30 cm ra khỏi trạm
3. Gửi nav goal bình thường

---

## Khắc phục sự cố

| Triệu chứng | Nguyên nhân | Giải pháp |
|---|---|---|
| rplidar không có `/scan` khi boot | USB chưa ổn định khi systemd start | `ExecStartPre=/bin/sleep 15` trong service (đã có) |
| Thiết bị USB timeout (ETIMEDOUT -110) | USB hub không có nguồn riêng | Dùng powered USB hub |
| App robot không kết nối được API | `API_HOST` sai | Sửa `setup/start_app_robot.sh` |
| DDS không thấy topic cross-machine | AP client isolation | Tắt trên router, kiểm tra `ROS_DOMAIN_ID` |
| line_follow không bám vạch | `SENSOR_GATE` / `THRESHOLD_SUM` sai | Điều chỉnh param trong `line_follow.py` |
| Nav2 không nhận goal | `atlas_api` chưa chạy hoặc nav2 chưa sẵn sàng | Kiểm tra `ros2 topic list`, `/navigate_to_pose` action |

---

## Biến môi trường

```bash
export ROS_DOMAIN_ID=4        # phải giống nhau trên PC và robot
export ROS_LOCALHOST_ONLY=0   # bắt buộc để DDS cross-machine hoạt động
```

---

## Liên hệ

roboticsvn.ai@gmail.com
