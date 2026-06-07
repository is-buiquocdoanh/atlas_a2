#!/usr/bin/env python3

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import serial

SENSOR_ID   = 1
START_REG   = 0x20   # 16-channel analog
REG_QTY     = 8      # 8 registers = 16 bytes of data

SER_TIMEOUT = 0.05
DE_RE_DELAY = 0.005
IO_HZ       = 50.0


def _crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)


def _build_req(slave_id: int, start_reg: int, qty: int) -> bytes:
    frame = bytes([
        slave_id, 0x03,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (qty >> 8) & 0xFF, qty & 0xFF,
    ])
    return frame + _crc16_modbus(frame)


def _check_crc(frame: bytes) -> bool:
    return len(frame) >= 5 and frame[-2:] == _crc16_modbus(frame[:-2])


def _read_exact(ser: serial.Serial, nbytes: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    data = b""
    while len(data) < nbytes and time.monotonic() < deadline:
        chunk = ser.read(nbytes - len(data))
        if chunk:
            data += chunk
    return data


class MagSensorNode(Node):
    """
    Reads 16-channel analog data from a single RS485 Modbus sensor (ID=1)
    and publishes to /sensor/analog16 at up to IO_HZ.

    Serial IO runs in a dedicated thread so the ROS2 event loop is never
    blocked waiting for hardware responses.
    """

    def __init__(self):
        super().__init__("mag_sensor_node")

        self.declare_parameter("port",    "/dev/magnetic")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baudrate").value)

        self._pub = self.create_publisher(UInt16MultiArray, "/sensor/analog16", 10)

        self._ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=8, parity='N', stopbits=1,
            timeout=SER_TIMEOUT,
        )
        try:
            self._ser.rs485_mode = serial.rs485.RS485Settings()
        except Exception:
            pass  # adapter handles DE/RE automatically or not supported

        self.get_logger().info(f"mag_sensor_node: {port} @ {baud} baud, target {IO_HZ} Hz")

        self._running = True
        self._io_thread = threading.Thread(
            target=self._io_loop, daemon=True, name="mag_io"
        )
        self._io_thread.start()

    # ------------------------------------------------------------------ #

    def _read_once(self) -> list:
        """One Modbus RTU request/response cycle. Returns 16 raw bytes or raises."""
        req = _build_req(SENSOR_ID, START_REG, REG_QTY)

        self._ser.reset_input_buffer()
        self._ser.write(req)
        self._ser.flush()
        time.sleep(DE_RE_DELAY)  # wait for RS485 adapter to switch DE→RE

        # Read 3-byte header, skipping leading 0xFF idle bytes emitted by
        # some RS485 adapters during the DE→RE direction switch.
        deadline = time.monotonic() + SER_TIMEOUT
        header = b""
        while time.monotonic() < deadline:
            b = self._ser.read(1)
            if not b:
                break
            if b[0] == 0xFF and not header:
                continue  # skip idle byte
            header += b
            if len(header) == 3:
                break

        if len(header) < 3:
            raise TimeoutError(f"short header ({len(header)} bytes)")

        bytecount = header[2]
        rest = _read_exact(self._ser, bytecount + 2, SER_TIMEOUT)
        if len(rest) < bytecount + 2:
            raise TimeoutError(f"short data ({len(rest)}/{bytecount+2})")

        resp = header + rest
        if not _check_crc(resp):
            raise ValueError(f"CRC error: {resp.hex(' ')}")
        if resp[0] != SENSOR_ID or resp[1] != 0x03 or resp[2] != REG_QTY * 2:
            raise ValueError(f"unexpected frame: {resp.hex(' ')}")

        return list(resp[3:3 + REG_QTY * 2])

    def _io_loop(self) -> None:
        """Dedicated thread: reads sensor at IO_HZ and publishes immediately."""
        period = 1.0 / IO_HZ
        err_count = 0

        while self._running:
            t0 = time.monotonic()

            try:
                data = self._read_once()
                err_count = 0
                msg = UInt16MultiArray()
                msg.data = [int(v) for v in data]
                self._pub.publish(msg)
            except Exception as e:
                err_count += 1
                if err_count % 10 == 1:
                    self.get_logger().warning(f"Sensor read error (×{err_count}): {e}")

            elapsed = time.monotonic() - t0
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ------------------------------------------------------------------ #

    def destroy_node(self):
        self._running = False
        self._io_thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MagSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
