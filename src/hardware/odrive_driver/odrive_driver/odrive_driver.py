from pathlib import Path

import odrive
import odrive.utils
import rclpy
import utils.config
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TwistWithCovariance, TwistWithCovarianceStamped, Vector3
from odrive.enums import AxisState, ControlMode, InputMode, ODriveError
from playsound import playsound
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Header

from .odrive_driver_config import OdriveDriverConfig


class OdriveDriver(Node):
    def __init__(self):
        super().__init__("odrive_driver")

        self.config = utils.config.load(self, OdriveDriverConfig)

        self.watchdog_triggered = False

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(TwistWithCovarianceStamped, "enc_vel", 10)

        self.create_timer(self.config.publish_period_s, self.publish_enc_vel)
        self.watchdog_timer = self.create_timer(self.config.cmd_vel_timeout_s, self.watchdog_callback)

        if self.config.debug:
            self.create_timer(0.01, self.log_debug_info)

    def init(self) -> bool:
        try:
            self.get_logger().info(f"Finding left ODrive (serial number {self.config.left_odrive.serial})...")
            self.odrive_left = odrive.find_any(serial_number=self.config.left_odrive.serial)
            self.initialize_odrive(self.odrive_left)
            self.get_logger().info("Found left ODrive")

            self.get_logger().info(f"Finding right ODrive (serial number {self.config.right_odrive.serial})...")
            self.odrive_right = odrive.find_any(serial_number=self.config.right_odrive.serial)
            self.initialize_odrive(self.odrive_right)
            self.get_logger().info("Found right ODrive")
            return True
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize ODrive: {e}")
            return False

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.watchdog_timer.reset()
        self.watchdog_triggered = False

        if not self.is_robot_enabled():
            self.get_logger().warning("Robot disabled", throttle_duration_sec=2.0)
            self.set_motor_rps(0.0, 0.0)
            return

        left_rps, right_rps = self.config.twist_to_motor_rps(msg.linear.x, msg.angular.z)
        self.set_motor_rps(left_rps, right_rps)

    def watchdog_callback(self) -> None:
        if not self.watchdog_triggered:
            self.get_logger().error("cmd_vel timed out, stopping robot")
            self.watchdog_triggered = True

        self.set_motor_rps(0.0, 0.0)

    def publish_enc_vel(self) -> None:
        left_motor_rps, right_motor_rps = self.get_motor_rps()
        linear_mps, angular_radps = self.config.motor_rps_to_twist(left_motor_rps, right_motor_rps)

        # Back-date to compensate for encoder read and processing latency
        timestamp_adjustment = self.get_clock().now() - Duration(nanoseconds=int(self.config.timestamp_delay_s * 1e9))
        self.publisher.publish(
            TwistWithCovarianceStamped(
                header=Header(stamp=timestamp_adjustment.to_msg(), frame_id=self.config.frame_id),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=linear_mps, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=angular_radps),
                    ),
                    covariance=self.config.twist_covariance(linear_mps, angular_radps),
                ),
            )
        )

    def initialize_odrive(self, odrv) -> None:
        odrv.clear_errors()
        try:
            odrive.utils.request_state(odrv.axis0, AxisState.CLOSED_LOOP_CONTROL)
        except Exception as e:
            if ODriveError.DC_BUS_UNDER_VOLTAGE in ODriveError(odrv.axis0.active_errors):
                playsound(Path(get_package_share_directory("odrive_driver")) / "sounds" / "turn_off_the_estop.wav")
                raise RuntimeError("EStop is engaged") from e
            raise
        odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
        odrv.axis0.controller.config.vel_gain = self.config.controller.vel_gain
        odrv.axis0.controller.config.vel_integrator_gain = self.config.controller.vel_integrator_gain
        odrv.axis0.controller.config.vel_integrator_limit = self.config.controller.vel_integrator_limit
        odrv.axis0.controller.config.vel_limit = self.config.vel_limit_motor
        odrv.axis0.controller.config.vel_ramp_rate = self.config.vel_ramp_rate_motor
        odrv.axis0.controller.config.inertia = self.config.inertia_motor

    def set_motor_rps(self, left_motor_rps: float, right_motor_rps: float) -> None:
        self.odrive_left.axis0.controller.input_vel = left_motor_rps
        self.odrive_right.axis0.controller.input_vel = right_motor_rps

    def get_motor_rps(self) -> tuple[float, float]:
        return self.odrive_left.axis0.vel_estimate, self.odrive_right.axis0.vel_estimate

    def is_robot_enabled(self) -> bool:
        try:
            with open(self.config.estop_file_path) as f:
                return f.read().strip() != "1"  # only "1" stops the robot, everything else is enabled
        except Exception:
            self.get_logger().error("EStop file not found", throttle_duration_sec=30.0)
            return True  # if the e-stop file doesn't exist / is corrupted we assume e-stop is off

    def log_debug_info(self) -> None:
        self.get_logger().info(
            f"left Iq_setpoint={self.odrive_left.axis0.motor.foc.Iq_setpoint:.3f} "
            f"Iq_measured={self.odrive_left.axis0.motor.foc.Iq_measured:.3f} "
            f"vel_setpoint={self.odrive_left.axis0.controller.vel_setpoint:.3f} "
            f"vel_estimate={self.odrive_left.axis0.vel_estimate:.3f} | "
            f"right Iq_setpoint={self.odrive_right.axis0.motor.foc.Iq_setpoint:.3f} "
            f"Iq_measured={self.odrive_right.axis0.motor.foc.Iq_measured:.3f} "
            f"vel_setpoint={self.odrive_right.axis0.controller.vel_setpoint:.3f} "
            f"vel_estimate={self.odrive_right.axis0.vel_estimate:.3f}",
        )


def main() -> None:
    rclpy.init()
    node = OdriveDriver()

    if not node.init():
        return

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
