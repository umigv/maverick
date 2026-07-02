import math

import utils.config
import utils.lifecycle
import utils.qos
from geometry_msgs.msg import Twist
from maverick_msgs.msg import MissionState
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from typing_extensions import assert_never
from utils.geometry import Path2d, Pose2d

from .controllers.controller import Controller
from .controllers.differential_drive.controller import DifferentialDriveController
from .controllers.pure_pursuit.controller import PurePursuitController
from .controllers.stanley.controller import StanleyController
from .path_tracking_config import PathTrackingConfig


class PathTracking(Node):
    def __init__(self) -> None:
        super().__init__("path_tracking")

        self.config: PathTrackingConfig = utils.config.load(self, PathTrackingConfig)

        match self.config.algorithm:
            case "pure_pursuit":
                self.controller: Controller = PurePursuitController(self.config.pure_pursuit, self.get_logger())
            case "stanley":
                self.controller = StanleyController(self.config.stanley, self.get_logger())
            case "differential_drive":
                self.controller = DifferentialDriveController(self.config.differential_drive, self.get_logger())
            case _ as unreachable:
                assert_never(unreachable)

        self.pose: Pose2d | None = None
        self.current_speed: float = 0.0
        self.mission_state: MissionState | None = None

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(Path, "path", self.path_callback, 10)
        self.create_subscription(MissionState, "mission_state", self.mission_state_callback, utils.qos.LATCHED)

        self.cmd_vel_publisher = self.create_publisher(Twist, "nav_cmd_vel", 10)

        self.create_timer(self.config.control_period_s, self.control_loop)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warning(
                f"Dropping odometry: frame '{msg.header.frame_id}' != '{self.config.odom_frame_id}'"
            )
            return

        if msg.child_frame_id != self.config.base_frame_id:
            self.get_logger().warning(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != '{self.config.base_frame_id}'"
            )
            return

        self.pose = Pose2d.from_ros(msg.pose.pose)
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def mission_state_callback(self, msg: MissionState) -> None:
        self.mission_state = msg

    def path_callback(self, path_msg: Path) -> None:
        if path_msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warning(
                f"Dropping path: frame '{path_msg.header.frame_id}' != '{self.config.odom_frame_id}'"
            )
            return

        try:
            path = Path2d.from_ros(path_msg)
        except ValueError as e:
            self.get_logger().warning(f"Dropping path: {e}")
            return

        self.get_logger().info("Received a new path from subscription.")
        self.controller.set_path(path)

    def control_loop(self) -> None:
        if self.pose is None:
            return

        if self.mission_state is not None and self.mission_state.mission_complete:
            return

        cmd = self.controller.compute_command(self.pose, self.current_speed)

        if cmd is not None:
            if self.mission_state is not None and self.mission_state.in_ramp_approach:
                cmd.linear.x = min(cmd.linear.x, self.config.ramp_max_speed_mps)
            self.cmd_vel_publisher.publish(cmd)


def main() -> None:
    utils.lifecycle.run_node(PathTracking)
