from math import hypot

import utils.lifecycle
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class PathHistory(Node):
    def __init__(self):
        super().__init__("path_history")
        self.actual_points: list[Point] = []
        self.cur_frame = None
        self.received_path = False
        self.received_odom = False

        self.planned_pub = self.create_publisher(Marker, "planned_path_points", 10)
        self.actual_pub = self.create_publisher(Marker, "actual_path_points", 10)

        self.create_subscription(Path, "smoothed_path", self.on_path, 10)
        self.create_subscription(Odometry, "odom/local", self.on_ondom, 10)
        self.create_timer(5.0, self.log_waiting_for_inputs)

        self.get_logger().info(
            f"Listening on {self.resolve_topic_name('smoothed_path')} and {self.resolve_topic_name('odom/local')}; "
            f"publishing {self.planned_pub.topic_name} and {self.actual_pub.topic_name}. "
            "Trail points are sampled every 0.5 m."
        )

    def log_waiting_for_inputs(self):
        missing = []
        if not self.received_path:
            missing.append(self.resolve_topic_name("smoothed_path"))
        if not self.received_odom:
            missing.append(self.resolve_topic_name("odom/local"))
        if missing:
            self.get_logger().info(f"Waiting for first message on: {', '.join(missing)}", throttle_duration_sec=15.0)

    def make_marker(self, header: Header, namespace: str, points: list[Point], color: ColorRGBA) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.points = points
        marker.color = color

        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0.2
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        return marker

    def on_path(self, msg: Path):
        first_message = not self.received_path
        self.received_path = True
        if first_message:
            self.get_logger().info(
                f"First planned path received: {len(msg.poses)} poses, frame={msg.header.frame_id!r}"
            )
        points: list[Point] = []
        for pose in msg.poses:
            points.append(Point(x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z))
        self.planned_pub.publish(
            self.make_marker(
                header=msg.header, namespace="planned", points=points, color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            )
        )
        self.get_logger().info(
            f"Published planned path: {len(points)} points, frame={msg.header.frame_id!r}",
            throttle_duration_sec=5.0,
        )
        if not points:
            self.get_logger().warning(
                "Planned path is empty; no planned dots will be visible.", throttle_duration_sec=5.0
            )

    def on_ondom(self, msg: Odometry):
        if not self.received_odom:
            self.received_odom = True
            self.get_logger().info(f"First odometry received: frame={msg.header.frame_id!r}")
        if msg.header.frame_id != self.cur_frame:
            if self.cur_frame is not None:
                self.get_logger().warning(
                    f"Odometry frame changed from {self.cur_frame!r} to {msg.header.frame_id!r}; clearing trail."
                )
            self.actual_points.clear()
            self.cur_frame = msg.header.frame_id

        position: Point = msg.pose.pose.position

        self.get_logger().info(
            f"Robot position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}; "
            f"frame={msg.header.frame_id!r}",
            throttle_duration_sec=5.0,
        )

        if self.actual_points:
            previous: Point = self.actual_points[-1]
            if hypot(position.x - previous.x, position.y - previous.y) < 0.5:
                return

        self.actual_points.append(position)
        self.actual_pub.publish(
            self.make_marker(
                header=msg.header,
                namespace="actual",
                points=self.actual_points,
                color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            )
        )
        self.get_logger().info(
            f"Published actual trail: {len(self.actual_points)} points; "
            f"marker subscribers={self.actual_pub.get_subscription_count()}",
            throttle_duration_sec=5.0,
        )


def main() -> None:
    utils.lifecycle.run_node(PathHistory)
