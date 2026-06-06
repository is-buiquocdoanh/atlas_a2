#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int32MultiArray

from motor_control import (
    LEFT_ID, RIGHT_ID, PORT, SERIAL_BAUD, GEAR_RATIO,
    set_gear_ratio, set_inversion, set_inversions,
    apply_inversion, twist_to_rpm,
    initialize_driver, run_speed_rpm, stop_motor, read_encoder,
)
from usb_can_a import USBCanA

_ENCODER_READ_TIMEOUT = 0.02   # 20ms per encoder read — hardware typically responds in 1-5ms
_ENCODER_HZ = 30.0             # encoder IO thread target rate


class TsdaDriver(Node):

    def __init__(self):
        super().__init__("tsda_driver")

        port_param   = self.declare_parameter("port",        PORT).get_parameter_value().string_value
        baud_param   = int(self.declare_parameter("baudrate",   int(SERIAL_BAUD)).get_parameter_value().integer_value)
        gear_param   = float(self.declare_parameter("gear_ratio", float(GEAR_RATIO)).get_parameter_value().double_value)
        invert_left  = bool(self.declare_parameter("invert_left",  False).get_parameter_value().bool_value)
        invert_right = bool(self.declare_parameter("invert_right", True).get_parameter_value().bool_value)

        try:
            set_gear_ratio(gear_param)
        except Exception:
            self.get_logger().warning(f"Invalid gear_ratio: {gear_param}")
        set_inversions(left=invert_left, right=invert_right)

        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        self.dev = USBCanA(port=port_param, baudrate=baud_param)

        try:
            initialize_driver(self.dev, LEFT_ID)
            initialize_driver(self.dev, RIGHT_ID)
        except Exception as exc:
            self.get_logger().warning(f"Driver init warning: {exc}")

        def _on_set_params(params):
            successful, reason = True, ''
            for p in params:
                try:
                    if p.name == 'gear_ratio':
                        set_gear_ratio(p.value)
                    elif p.name == 'invert_left':
                        set_inversion(LEFT_ID, bool(p.value))
                    elif p.name == 'invert_right':
                        set_inversion(RIGHT_ID, bool(p.value))
                except Exception as exc:
                    successful, reason = False, str(exc)
            return SetParametersResult(successful=successful, reason=reason)

        self.add_on_set_parameters_callback(_on_set_params)

        self.vx = 0.0
        self.omega = 0.0
        self.control_active = False
        self._vel_deadzone = 1e-4

        # Lock protecting self.dev — shared between motor timer and encoder IO thread
        self._serial_lock = threading.Lock()

        # Encoder publisher
        self._encoder_pub = self.create_publisher(Int32MultiArray, "/encoder/raw", 10)

        # Encoder IO thread: reads encoders independently, publishes /encoder/raw
        self._running = True
        self._enc_thread = threading.Thread(
            target=self._encoder_loop, daemon=True, name="encoder_io"
        )
        self._enc_thread.start()

        self.timer = self.create_timer(0.01, self.update)  # 100 Hz motor control

    # ------------------------------------------------------------------ #
    #  Encoder IO thread                                                   #
    # ------------------------------------------------------------------ #

    def _encoder_loop(self) -> None:
        """Reads both encoders at _ENCODER_HZ and publishes /encoder/raw.

        Uses a lock shared with the motor-command timer so the two never
        access the serial port simultaneously. Before each read the RX
        buffer is flushed to discard stale motor-command responses.
        """
        period = 1.0 / _ENCODER_HZ
        while self._running:
            t0 = time.monotonic()

            with self._serial_lock:
                try:
                    self.dev.ser.reset_input_buffer()
                except Exception:
                    pass
                left  = read_encoder(self.dev, LEFT_ID,  response_timeout=_ENCODER_READ_TIMEOUT)
                right = read_encoder(self.dev, RIGHT_ID, response_timeout=_ENCODER_READ_TIMEOUT)

            if left is not None and right is not None:
                msg = Int32MultiArray()
                msg.data = [left, right]
                self._encoder_pub.publish(msg)

            elapsed = time.monotonic() - t0
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ------------------------------------------------------------------ #
    #  ROS2 callbacks                                                      #
    # ------------------------------------------------------------------ #

    def cmd_callback(self, msg):
        vx    = msg.linear.x
        omega = msg.angular.z
        self.vx    = vx
        self.omega = omega

        nonzero = (abs(vx) > self._vel_deadzone) or (abs(omega) > self._vel_deadzone)

        if nonzero and not self.control_active:
            self.get_logger().debug("Control active")
            self.control_active = True

        if not nonzero and self.control_active:
            self.get_logger().info("Control inactive: sending stop sequence")
            try:
                with self._serial_lock:
                    for _ in range(3):
                        run_speed_rpm(self.dev, LEFT_ID,  0, wait_response=False)
                        run_speed_rpm(self.dev, RIGHT_ID, 0, wait_response=False)
                        time.sleep(0.02)
                    stop_motor(self.dev, LEFT_ID)
                    stop_motor(self.dev, RIGHT_ID)
            except Exception as exc:
                self.get_logger().warning(f"Stop sequence failed: {exc}")
            self.control_active = False

    def update(self):
        rpm_left, rpm_right = twist_to_rpm(self.vx, self.omega)

        if self.control_active:
            with self._serial_lock:
                run_speed_rpm(self.dev, LEFT_ID,  apply_inversion(LEFT_ID,  rpm_left),  wait_response=False)
                run_speed_rpm(self.dev, RIGHT_ID, apply_inversion(RIGHT_ID, rpm_right), wait_response=False)


def main(args=None):
    rclpy.init(args=args)
    node = TsdaDriver()

    try:
        rclpy.spin(node)
    finally:
        node._running = False
        node._enc_thread.join(timeout=1.0)

        try:
            with node._serial_lock:
                for _ in range(4):
                    run_speed_rpm(node.dev, LEFT_ID,  0, wait_response=False)
                    run_speed_rpm(node.dev, RIGHT_ID, 0, wait_response=False)
                    time.sleep(0.02)
                stop_motor(node.dev, LEFT_ID)
                stop_motor(node.dev, RIGHT_ID)
        except Exception:
            pass

        try:
            node.dev.close()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
