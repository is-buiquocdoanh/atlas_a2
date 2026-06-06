#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from motor_control import read_encoder, LEFT_ID, RIGHT_ID, PORT, SERIAL_BAUD
from usb_can_a import USBCanA

# Short timeout for real-time reads — hardware typically responds in 1-5ms
_READ_TIMEOUT = 0.02


class EncoderRawNode(Node):
    """
    Publishes raw encoder counts (left, right) at high rate.

    Serial IO runs in a dedicated thread so the ROS2 event loop is never
    blocked waiting for hardware responses. The ROS timer only publishes
    the latest values already captured by the IO thread.

    Topic: /encoder/raw  (std_msgs/Int32MultiArray)
      data[0] = left encoder count (absolute, signed 32-bit)
      data[1] = right encoder count (absolute, signed 32-bit)
    """

    def __init__(self):
        super().__init__("encoder_raw_node")

        port = self.declare_parameter("port", PORT).get_parameter_value().string_value
        baud = int(self.declare_parameter("baudrate", int(SERIAL_BAUD)).get_parameter_value().integer_value)
        pub_hz = self.declare_parameter("pub_hz", 50.0).get_parameter_value().double_value
        read_hz = self.declare_parameter("read_hz", 100.0).get_parameter_value().double_value

        self.get_logger().info(
            f"encoder_raw_node: port={port} baud={baud} "
            f"read_hz={read_hz} pub_hz={pub_hz}"
        )

        # USBCanA opened with short timeout so IO thread never hangs long
        self.dev = USBCanA(port=port, baudrate=baud, timeout=_READ_TIMEOUT)

        self._pub = self.create_publisher(Int32MultiArray, "/encoder/raw", 10)

        # Shared state: IO thread writes, ROS timer reads
        self._lock = threading.Lock()
        self._left: Optional[int] = None
        self._right: Optional[int] = None
        self._io_errors = 0

        # IO thread reads encoders at read_hz
        self._read_period = 1.0 / read_hz
        self._running = True
        self._io_thread = threading.Thread(target=self._io_loop, daemon=True, name="encoder_io")
        self._io_thread.start()

        # Publish timer — only pushes to ROS, no blocking IO here
        self.create_timer(1.0 / pub_hz, self._publish)

    def _io_loop(self) -> None:
        """Dedicated thread: reads both encoders then sleeps to hit target rate."""
        while self._running:
            t0 = time.monotonic()

            left = read_encoder(self.dev, LEFT_ID, response_timeout=_READ_TIMEOUT)
            right = read_encoder(self.dev, RIGHT_ID, response_timeout=_READ_TIMEOUT)

            if left is not None and right is not None:
                with self._lock:
                    self._left = left
                    self._right = right
                self._io_errors = 0
            else:
                self._io_errors += 1
                if self._io_errors % 10 == 1:
                    self.get_logger().warning(
                        f"Encoder read failed (left={left}, right={right}), "
                        f"error count={self._io_errors}"
                    )

            elapsed = time.monotonic() - t0
            sleep_t = self._read_period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _publish(self) -> None:
        """ROS2 timer callback — publishes latest encoder values, no IO here."""
        with self._lock:
            left = self._left
            right = self._right

        if left is None or right is None:
            return

        msg = Int32MultiArray()
        msg.data = [left, right]
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._running = False
        self._io_thread.join(timeout=1.0)
        try:
            self.dev.close()
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = EncoderRawNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
