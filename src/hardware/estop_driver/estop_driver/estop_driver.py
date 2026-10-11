from typing import override

import serial
import utils.config
import utils.lifecycle
import utils.qos
import utils.watchdog
from rclpy.node import Node
from std_msgs.msg import Bool

from .estop_driver_config import EstopDriverConfig


class EstopDriver(Node):
    def __init__(self) -> None:
        super().__init__("estop_driver")

        self.config = utils.config.load(self, EstopDriverConfig)

        self.publisher = self.create_publisher(Bool, "estop", utils.qos.LATCHED)
        self.current_state = "1"
        self.publish_estop_state()
        self.serial_watchdog = utils.watchdog.Watchdog(self, self.config.serial_timeout_s, self.serial_timeout_callback)
        self.read_buffer: bytes = b""
        self.serial: serial.Serial | None = None

        try:
            self.serial = serial.Serial(str(self.config.serial_port), baudrate=self.config.baud_rate, timeout=0.0)
            self.get_logger().info(f"Connected to {self.config.serial_port}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Failed to connect to {self.config.serial_port}: {e}")
            raise SystemExit(1) from None

        self.create_timer(self.config.poll_period_s, self.poll)
        self.create_timer(self.config.poll_period_s, self.publish_estop_state)

    def poll(self) -> None:
        assert self.serial is not None

        if self.serial.in_waiting <= 0:
            return

        self.read_buffer += self.serial.read(self.serial.in_waiting)
        while b"\n" in self.read_buffer:
            raw, _, self.read_buffer = self.read_buffer.partition(b"\n")
            line = raw.decode("utf-8", errors="ignore").strip()

            if line not in ("0", "1"):
                self.get_logger().warning(f"Unexpected estop line: {line!r}", throttle_duration_sec=2.0)
                continue
            self.serial_watchdog.pet()
            if line == self.current_state:
                continue

            self.current_state = line
            self.publish_estop_state()
            state_str = "STOP" if line == "1" else "SAFE"
            self.get_logger().info(f"State changed: {line} -> {state_str}")

    def publish_estop_state(self) -> None:
        self.publisher.publish(Bool(data=self.current_state == "1"))

    def serial_timeout_callback(self) -> None:
        self.get_logger().error("EStop serial state timed out, stopping robot", throttle_duration_sec=2.0)
        self.current_state = "1"
        self.publish_estop_state()

    @override
    def destroy_node(self) -> None:
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main() -> None:
    utils.lifecycle.run_node(EstopDriver)
