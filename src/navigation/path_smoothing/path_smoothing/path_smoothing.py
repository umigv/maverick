import rclpy
import utils.config
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from utils.geometry import Point2d

from .path_smoothing_config import PathSmoothingConfig


class PathSmoother(Node):
    def __init__(self) -> None:
        super().__init__("path_smoothing")

        self.config: PathSmoothingConfig = utils.config.load(self, PathSmoothingConfig)

        self.create_subscription(Path, "path", self.path_callback, 10)

        self.path_publisher = self.create_publisher(Path, "smoothed_path", 10)

    def path_callback(self, msg: Path) -> None:
        points = [Point2d.from_ros(ps.pose.position) for ps in msg.poses]
        smoothed = self.smooth_chaikin(points)

        self.path_publisher.publish(
            Path(
                header=msg.header,
                poses=[
                    PoseStamped(
                        header=msg.header,
                        pose=Pose(position=point.to_ros(), orientation=Quaternion(w=1.0)),
                    )
                    for point in smoothed
                ],
            )
        )

    def smooth_chaikin(self, path: list[Point2d]) -> list[Point2d]:
        """Chaikin corner cutting: each iteration inserts points at 1/4 and 3/4 of every segment,
        rounding sharp corners while keeping the start and end waypoints fixed.
        https://www.cs.unc.edu/~dm/UNC/COMP258/LECTURES/Chaikins-Algorithm.pdf
        """
        if len(path) < 2 or self.config.chaikin_iterations == 0:
            return path

        points = path
        for _ in range(self.config.chaikin_iterations):
            smoothed = [points[0]]

            for i in range(len(points) - 1):
                smoothed.append(points[i].lerp(points[i + 1], 0.25))
                smoothed.append(points[i].lerp(points[i + 1], 0.75))

            smoothed.append(points[-1])
            points = smoothed
        return points


def main() -> None:
    rclpy.init()
    node = PathSmoother()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
