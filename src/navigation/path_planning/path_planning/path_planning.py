import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support
import utils.config
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from utils.geometry import Point2d, Pose2d
from utils.world_occupancy_grid import WorldOccupancyGrid

from .path_planning_config import PathPlanningConfig
from .path_planning_impl import generate_path, pull_string


class PathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("path_planning")

        self.config: PathPlanningConfig = utils.config.load(self, PathPlanningConfig)

        self.robot_pose: Pose2d | None = None
        self.grid: WorldOccupancyGrid | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(PointStamped, "goal", self.goal_callback, 10)

        self.path_publisher = self.create_publisher(Path, "path", 10)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config frame ID ({self.config.frame_id})"
            )
            return

        self.robot_pose = Pose2d.from_ros(msg.pose.pose)

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.config.frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config frame ID ({self.config.frame_id})"
            )
            return

        self.grid = WorldOccupancyGrid(msg)

    def goal_callback(self, msg: PointStamped) -> None:
        if self.robot_pose is None or self.grid is None:
            return

        # Set timestamp to 0 so tf uses the latest available transform instead of looking up the exact time. Without
        # this, a slightly future timestamp would cause tf to throw a "requested time is in the future" error.
        #
        # The real fix would be adding a timeout to transform() and switching to a MultiThreadedExecutor so tf_listener
        # can update while we block, but that introduces thread-safety concerns. Using stamp=0 is acceptable because 5ms
        # of delay corresponds to only ~5mm of position error at 1 m/s.
        msg.header.stamp = rclpy.time.Time().to_msg()

        try:
            msg = self.tf_buffer.transform(msg, self.config.frame_id)
        except Exception as e:
            self.get_logger().error(f"Failed to transform goal to {self.config.frame_id}: {e}")
            return

        path_points = generate_path(
            self.grid,
            self.robot_pose,
            Point2d.from_ros(msg.point),
            self.config,
        )

        if path_points is None:
            self.get_logger().warn("No path found to goal")
            return

        pulled = pull_string(
            self.grid,
            path_points,
            self.robot_pose,
            self.config,
        )

        header = Header(frame_id=self.config.frame_id, stamp=self.get_clock().now().to_msg())
        self.path_publisher.publish(
            Path(
                header=header,
                poses=[
                    PoseStamped(
                        header=header,
                        pose=Pose(position=point.to_ros(), orientation=Quaternion(w=1.0)),
                    )
                    for point in pulled
                ],
            )
        )


def main() -> None:
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
