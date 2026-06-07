#!/usr/bin/env python3
"""
Line follower dùng /sensor/analog16 từ mag_sensor_node (16 kênh analog 8-bit).

Thuật toán bám vạch mượt:
  1. Sensor gate     — loại bỏ nhiễu nền từng kênh trước khi tính centroid
  2. Sum threshold   — chỉ điều khiển khi tổng tín hiệu đủ mạnh (có vạch)
  3. Low-pass filter — làm mượt error, tránh giật do nhiễu tức thời
  4. PD control      — P bám vạch, D chống vọt lố
  5. Speed reduction — giảm tốc tiến tỉ lệ độ lệch, robot đủ thời gian quay
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32
from geometry_msgs.msg import Twist

# ── Thông số (ghi đè qua ROS parameter) ──────────────────────────────────────
LINEAR_X          = -0.15   # Tốc độ tiến (m/s) — giảm nếu robot lệch nhiều
KP                = 0.35   # Hệ số P — tăng nếu bám không sát, giảm nếu dao động
KD                = 0.08   # Hệ số D — tăng nếu còn vọt lố, giảm nếu phản ứng trễ
LPF_ALPHA         = 0.4    # Low-pass: gần 0=lọc mạnh (chậm), gần 1=ít lọc (nhanh)
SENSOR_GATE       = 10     # Ngưỡng từng kênh: bỏ qua sensor < giá trị này (lọc nhiễu nền)
THRESHOLD_SUM     = 80     # Tổng tín hiệu tối thiểu để nhận ra có vạch — QUAN TRỌNG
                           #   trên vạch ≈ 400-500, ngoài vạch ≈ 40 → đặt ~80
MAX_ANG           = 1.2    # Vận tốc góc tối đa (rad/s)
LOST_TIMEOUT      = 0.3    # Mất vạch quá lâu này (s) → dừng hẳn
CONTROL_RATE      = 20.0   # Tần số điều khiển (Hz)
SPEED_REDUCE      = 0.5    # Giảm tốc tiến theo độ lệch: 0=không giảm, 1=dừng khi lệch max

# Vị trí logic của 16 kênh (trái âm, phải dương, đơn vị tùy ý)
_POSITIONS = [-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5,
               0.5,  1.5,  2.5,  3.5,  4.5,  5.5,  6.5,  7.5]
_MAX_POS = 7.5


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follow")

        self.declare_parameter("linear_x",      LINEAR_X)
        self.declare_parameter("kp",            KP)
        self.declare_parameter("kd",            KD)
        self.declare_parameter("lpf_alpha",     LPF_ALPHA)
        self.declare_parameter("sensor_gate",   float(SENSOR_GATE))
        self.declare_parameter("threshold_sum", float(THRESHOLD_SUM))
        self.declare_parameter("max_ang",       MAX_ANG)
        self.declare_parameter("lost_timeout",  LOST_TIMEOUT)
        self.declare_parameter("control_rate",  CONTROL_RATE)
        self.declare_parameter("speed_reduce",  SPEED_REDUCE)

        self.linear_x      = self.get_parameter("linear_x").value
        self.kp            = self.get_parameter("kp").value
        self.kd            = self.get_parameter("kd").value
        self.lpf_alpha     = self.get_parameter("lpf_alpha").value
        self.sensor_gate   = self.get_parameter("sensor_gate").value
        self.threshold_sum = self.get_parameter("threshold_sum").value
        self.max_ang       = self.get_parameter("max_ang").value
        self.lost_timeout  = self.get_parameter("lost_timeout").value
        self.control_rate  = self.get_parameter("control_rate").value
        self.speed_reduce  = self.get_parameter("speed_reduce").value

        self._sub = self.create_subscription(
            UInt16MultiArray, "/sensor/analog16", self._cb_sensor, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_mag", 10)
        self._err_pub = self.create_publisher(Float32, "/line_error", 10)

        self._raw    = None
        self._last_t = 0.0

        self._filtered_err  = 0.0
        self._prev_err      = 0.0
        self._prev_err_time = time.monotonic()

        self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.get_logger().info(
            f"line_follow: sensor_gate={self.sensor_gate}, "
            f"threshold_sum={self.threshold_sum}, kp={self.kp}, kd={self.kd}"
        )

    def _cb_sensor(self, msg: UInt16MultiArray):
        self._raw    = list(msg.data)
        self._last_t = time.monotonic()

    def _compute_error(self):
        """
        Weighted centroid của 16 kênh sau khi lọc nhiễu nền.
        Trả về (error, sum_filtered) hoặc None nếu không thấy vạch.
        """
        data = self._raw
        if data is None or len(data) != 16:
            return None

        # Lọc nhiễu nền: sensor dưới gate coi là 0
        gated = [v if v >= self.sensor_gate else 0 for v in data]

        total = sum(gated)
        if total < self.threshold_sum:
            return None  # không đủ tín hiệu → không có vạch

        error = sum(p * v for p, v in zip(_POSITIONS, gated)) / total
        return error

    def _control_loop(self):
        now = time.monotonic()
        cmd = Twist()

        # Dừng nếu chưa nhận được dữ liệu hoặc mất vạch quá lâu
        if self._raw is None or (now - self._last_t) > self.lost_timeout:
            self._cmd_pub.publish(cmd)
            return

        error = self._compute_error()
        if error is None:
            # Không thấy vạch → dừng hẳn, reset bộ lọc
            self._filtered_err = 0.0
            self._prev_err = 0.0
            self._cmd_pub.publish(cmd)
            return

        # 1. Low-pass filter
        a = self.lpf_alpha
        self._filtered_err = a * error + (1.0 - a) * self._filtered_err

        # 2. D term
        dt = now - self._prev_err_time
        d_err = (self._filtered_err - self._prev_err) / dt if dt > 0.001 else 0.0
        self._prev_err      = self._filtered_err
        self._prev_err_time = now

        # 3. PD → vận tốc góc (dấu âm vì error dương = lệch phải → quay trái)
        angular = -(self.kp * self._filtered_err + self.kd * d_err)
        angular = max(-self.max_ang, min(self.max_ang, angular))

        # 4. Giảm tốc tiến tỉ lệ độ lệch
        err_norm = min(abs(self._filtered_err) / _MAX_POS, 1.0)
        linear   = self.linear_x * (1.0 - self.speed_reduce * err_norm)

        cmd.linear.x  = float(linear)
        cmd.angular.z = float(angular)
        self._cmd_pub.publish(cmd)

        msg = Float32()
        msg.data = float(self._filtered_err)
        self._err_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
