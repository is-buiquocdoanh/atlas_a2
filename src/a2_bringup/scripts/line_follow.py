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
LINEAR_X      = -0.05
KP            = 0.007
KD            = 0.05
LPF_ALPHA     = 0.30
SENSOR_GATE   = 10
THRESHOLD_SUM = 60
MAX_ANG       = 0.30
LOST_TIMEOUT  = 0.3
CONTROL_RATE  = 20.0
SPEED_REDUCE  = 0.70
NO_LINE_STOP  = 0.5    # mất vạch khi đang FOLLOW → dừng hẳn (s)

# ── Search params ─────────────────────────────────────────────────────────────
INIT_WAIT       = 0.5   # chờ sensor ổn định khi mới khởi động (s)
SEARCH_BACK_T   = 2.0   # bước 0: lùi ~10cm (LINEAR_X=-0.05 × 2.0s = 10cm)
SEARCH_ANG      = 0.13  # tốc độ quay khi tìm vạch (rad/s)
SEARCH_LEFT_T   = 4.0   # bước 1: xoay TRÁI ~30°
SEARCH_RIGHT_T  = 4.0   # bước 2: xoay PHẢI ~30° (qua tâm)
SEARCH_RETURN_T = 2.0   # bước 3: về GIỮA

# (linear, angular_sign) cho từng bước search
# linear=1 → dùng self.linear_x; linear=0 → đứng yên chỉ xoay
_SEARCH_STEPS = [
    (SEARCH_BACK_T,   1,    0.0),   # bước 0: lùi thẳng
    (SEARCH_LEFT_T,   0,   +1.0),   # bước 1: xoay trái
    (SEARCH_RIGHT_T,  0,   -1.0),   # bước 2: xoay phải
    (SEARCH_RETURN_T, 0,   +1.0),   # bước 3: về giữa
]

# Vị trí logic của 16 kênh (trái âm, phải dương, đơn vị tùy ý)
_POSITIONS = [-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5,
               0.5,  1.5,  2.5,  3.5,  4.5,  5.5,  6.5,  7.5]
_MAX_POS = 7.5

# States
_INIT    = 0
_SEARCH  = 1
_FOLLOW  = 2
_STOPPED = 3


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
        self.declare_parameter("no_line_stop",  NO_LINE_STOP)

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
        self.no_line_stop  = self.get_parameter("no_line_stop").value

        self._sub     = self.create_subscription(
            UInt16MultiArray, "/sensor/analog16", self._cb_sensor, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_mag", 10)
        self._err_pub = self.create_publisher(Float32, "/line_error", 10)

        self._raw    = None
        self._last_t = 0.0

        self._filtered_err  = 0.0
        self._prev_err      = 0.0
        self._prev_err_time = time.monotonic()

        self._state         = _INIT
        self._state_t       = time.monotonic()
        self._search_step   = 0
        self._search_step_t = 0.0
        self._no_line_since = None

        self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.get_logger().info(
            f"line_follow: kp={self.kp}, kd={self.kd}, "
            f"sensor_gate={self.sensor_gate}, threshold_sum={self.threshold_sum}"
        )

    def _cb_sensor(self, msg: UInt16MultiArray):
        self._raw    = list(msg.data)
        self._last_t = time.monotonic()

    def _compute_error(self):
        now = time.monotonic()
        if self._raw is None or (now - self._last_t) > self.lost_timeout:
            return None
        gated = [max(0.0, float(v) - self.sensor_gate) for v in self._raw]
        total = sum(gated)
        if total < self.threshold_sum:
            return None
        return sum(p * v for p, v in zip(_POSITIONS, gated)) / total

    def _enter_search(self, now):
        self._state         = _SEARCH
        self._state_t       = now
        self._search_step   = 0
        self._search_step_t = now
        self.get_logger().info("vạch không thấy — bắt đầu quét tìm")

    def _control_loop(self):
        now   = time.monotonic()
        cmd   = Twist()
        error = self._compute_error()

        # ── STOPPED ───────────────────────────────────────────────────────────
        if self._state == _STOPPED:
            self._cmd_pub.publish(cmd)
            return

        # ── INIT ──────────────────────────────────────────────────────────────
        if self._state == _INIT:
            if now - self._state_t < INIT_WAIT:
                self._cmd_pub.publish(cmd)
                return
            if error is not None:
                self._state = _FOLLOW
                self.get_logger().info("bắt vạch ngay — FOLLOW")
            else:
                self._enter_search(now)
            return

        # ── SEARCH ────────────────────────────────────────────────────────────
        if self._state == _SEARCH:
            if error is not None:
                self._state = _FOLLOW
                self._no_line_since = None
                self._filtered_err  = error
                self._prev_err      = error
                self._prev_err_time = now
                self.get_logger().info(f"tìm thấy vạch ở bước {self._search_step} — FOLLOW")
                self._cmd_pub.publish(cmd)
                return

            if self._search_step >= len(_SEARCH_STEPS):
                self._state = _STOPPED
                self.get_logger().warn("không tìm thấy vạch — dừng hẳn")
                self._cmd_pub.publish(cmd)
                return

            duration, use_linear, ang_sign = _SEARCH_STEPS[self._search_step]

            if now - self._search_step_t >= duration:
                self._search_step  += 1
                self._search_step_t = now
                self._cmd_pub.publish(cmd)
                return

            cmd.linear.x  = float(self.linear_x) if use_linear else 0.0
            cmd.angular.z = ang_sign * SEARCH_ANG
            self._cmd_pub.publish(cmd)
            return

        # ── FOLLOW ────────────────────────────────────────────────────────────
        if self._state == _FOLLOW:
            if error is None:
                if self._no_line_since is None:
                    self._no_line_since = now
                elif now - self._no_line_since >= self.no_line_stop:
                    self._state = _STOPPED
                    self.get_logger().info("mất vạch — dừng hẳn")
                self._filtered_err = 0.0
                self._prev_err     = 0.0
                self._cmd_pub.publish(cmd)
                return

            self._no_line_since = None

            a = self.lpf_alpha
            self._filtered_err = a * error + (1.0 - a) * self._filtered_err

            dt    = now - self._prev_err_time
            d_err = (self._filtered_err - self._prev_err) / dt if dt > 0.001 else 0.0
            self._prev_err      = self._filtered_err
            self._prev_err_time = now

            angular  = -(self.kp * self._filtered_err + self.kd * d_err)
            angular  = max(-self.max_ang, min(self.max_ang, angular))
            err_norm = min(abs(self._filtered_err) / _MAX_POS, 1.0)
            linear   = self.linear_x * (1.0 - self.speed_reduce * err_norm)

            cmd.linear.x  = float(linear)
            cmd.angular.z = float(angular)
            self._cmd_pub.publish(cmd)

            msg      = Float32()
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
