import time

import serial
import utils.config
import utils.lifecycle
import utils.qos
from geometry_msgs.msg import Twist
from maverick_msgs.msg import MissionState
from rclpy.node import Node
from rclpy.time import Time

from .led_driver_config import LedDriverConfig


class LedDriver(Node):
    LED_ESTOP = 6  # Flashing red
    LED_TELEOP = 1  # Solid blue
    LED_NORMAL = 2  # Flashing blue
    LED_NO_MANS_LAND = 3  # Flashing green
    LED_RECOVERY = 4  # Flashing yellow
    LED_RAMP = 5  # Flashing purple
    LED_MISSION_COMPLETE = 7  # Solid green
    LED_UNKNOWN = 9  # Rainbow

    def __init__(self) -> None:
        super().__init__("led_driver")

        self.config = utils.config.load(self, LedDriverConfig)

        self.create_subscription(Twist, "teleop_cmd_vel", self.teleop_callback, 10)
        self.create_subscription(Twist, "nav_cmd_vel", self.nav_callback, 10)
        self.create_subscription(MissionState, "mission_state", self.mission_state_callback, utils.qos.LATCHED)

        self.last_teleop_time: Time | None = None
        self.last_nav_time: Time | None = None
        self.mission_state: MissionState | None = None
        self.last_sent: int | None = None
        self.serial: serial.Serial | None = None

        try:
            self.serial = serial.Serial(str(self.config.serial_port), baudrate=self.config.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.config.serial_port}")
            self.wait_for_ready()
        except (serial.SerialException, RuntimeError) as e:
            self.get_logger().fatal(f"Failed to connect to {self.config.serial_port}: {e}")
            raise SystemExit(1) from None

        self.create_timer(self.config.update_period_s, self.update)

    def wait_for_ready(self) -> None:
        assert self.serial is not None

        deadline = time.monotonic() + self.config.ready_timeout_s
        while time.monotonic() < deadline:
            line = self.serial.readline().decode(errors="ignore").strip()
            if line == "READY":
                self.get_logger().info("READY received")
                return

        raise RuntimeError(f"Did not receive READY within {self.config.ready_timeout_s}s")

    def teleop_callback(self, msg: Twist) -> None:
        self.last_teleop_time = self.get_clock().now()

    def nav_callback(self, msg: Twist) -> None:
        self.last_nav_time = self.get_clock().now()

    def mission_state_callback(self, msg: MissionState) -> None:
        self.mission_state = msg

    def is_robot_enabled(self) -> bool:
        try:
            with self.config.estop_file_path.open() as f:
                return f.read().strip() != "1"  # only "1" stops the robot, everything else is enabled
        except Exception:
            return True  # if the e-stop file doesn't exist / is corrupted we assume e-stop is off

    def is_active(self, last_time: Time | None) -> bool:
        if last_time is None:
            return False

        elapsed: float = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return elapsed <= self.config.cmd_vel_timeout_s

    def mission_led_value(self) -> int:
        if self.mission_state is None:
            # In modes without mission control the nav stack still drives the robot autonomously
            return self.LED_NORMAL if self.is_active(self.last_nav_time) else self.LED_UNKNOWN
        if self.mission_state.mission_complete:
            return self.LED_MISSION_COMPLETE
        if self.mission_state.in_recovery:
            return self.LED_RECOVERY
        if self.mission_state.in_ramp_approach:
            return self.LED_RAMP
        if self.mission_state.in_no_mans_land:
            return self.LED_NO_MANS_LAND
        return self.LED_NORMAL

    def update(self) -> None:
        if not self.is_robot_enabled():
            value = self.LED_ESTOP
        elif self.is_active(self.last_teleop_time):
            value = self.LED_TELEOP
        else:
            value = self.mission_led_value()

        self.send_led_value(value)

    def send_led_value(self, value: int) -> None:
        assert self.serial is not None

        if not self.serial.is_open:
            return

        try:
            self.serial.write(f"{value}\n".encode())
            if value != self.last_sent:
                self.get_logger().info(f"Sent: {value}")
            self.last_sent = value
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def destroy_node(self) -> None:
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main() -> None:
    utils.lifecycle.run_node(LedDriver)
