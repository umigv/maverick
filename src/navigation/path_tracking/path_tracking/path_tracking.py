import math

import rclpy
import utils.config
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from utils.geometry import Path2d, Pose2d

from .path_tracking_config import PathTrackingConfig
from .pure_pursuit_controller import PurePursuitController
from .stanley_controller import StanleyController


class PathTracking(Node):
    def __init__(self) -> None:
        super().__init__("path_tracking")

        self.config: PathTrackingConfig = utils.config.load(self, PathTrackingConfig)

        if self.config.algorithm == "pure_pursuit":
            self.controller: PurePursuitController | StanleyController = PurePursuitController(
                self.config.pure_pursuit, self.get_logger()
            )
        else:
            self.controller = StanleyController(self.config.stanley, self.get_logger())

        self.pose: Pose2d | None = None
        self.current_speed: float = 0.0

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(Path, "path", self.path_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, "nav_cmd_vel", 10)

        self.create_timer(self.config.control_period_s, self.control_loop)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warn(f"Dropping odometry: frame '{msg.header.frame_id}' != '{self.config.odom_frame_id}'")
            return

        if msg.child_frame_id != self.config.base_frame_id:
            self.get_logger().warn(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != '{self.config.base_frame_id}'"
            )
            return

        self.pose = Pose2d.from_ros(msg.pose.pose)
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def path_callback(self, path_msg: Path) -> None:
        if path_msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warn(
                f"Dropping path: frame '{path_msg.header.frame_id}' != '{self.config.odom_frame_id}'"
            )
            return

        try:
            path = Path2d.from_ros(path_msg)
        except ValueError as e:
            self.get_logger().warn(f"Dropping path: {e}")
            return

        self.get_logger().info("Received a new path from subscription.")
        self.controller.set_path(path)

    def control_loop(self) -> None:
        if self.pose is None:
            return

        cmd = self.controller.compute_command(self.pose, self.current_speed)

        if cmd is not None:
            self.cmd_vel_publisher.publish(cmd)


def main() -> None:
    rclpy.init()
    node = PathTracking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
