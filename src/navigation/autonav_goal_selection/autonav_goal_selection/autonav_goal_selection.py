import json
from dataclasses import dataclass

import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform
import tf2_ros
import utils.config
import utils.qos
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_msgs.msg import ColorRGBA, Header, String
from std_srvs.srv import SetBool
from tf2_ros import TransformException
from utils.geometry import Point2d, Pose2d
from utils.world_occupancy_grid import WorldOccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from .autonav_goal_selection_config import AutonavGoalSelectionConfig
from .autonav_goal_selection_impl import GoalSelector


@dataclass(frozen=True)
class Waypoint:
    point: Point2d
    no_mans_land: bool = False


class AutonavGoalSelection(Node):
    def __init__(self) -> None:
        super().__init__("autonav_goal_selection")

        self.config: AutonavGoalSelectionConfig = utils.config.load(self, AutonavGoalSelectionConfig)

        self.robot_pose: Pose2d | None = None
        self.grid: WorldOccupancyGrid | None = None
        self.goal_selector = GoalSelector(self.config.goal_selection_params)
        self.in_no_mans_land: bool = False
        self.state: string = "normal"

        self.waypoints: list[Waypoint] = self.load_waypoints()
        self.current_waypoint_index = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.set_recovery_client = self.create_client(SetBool, "state/set_recovery")
        self.set_no_mans_land_client = self.create_client(SetBool, "state/set_no_mans_land")

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(String, "state", self.state_callback, utils.qos.LATCHED)

        self.goal_publisher = self.create_publisher(PointStamped, "goal", 10)
        self.waypoint_publisher = self.create_publisher(PointStamped, "waypoint", utils.qos.LATCHED)
        self.debug_publisher = self.create_publisher(MarkerArray, "goal_selection_debug", 10)

        self.create_timer(self.config.goal_publish_period_s, self.publish_goal)

        self.publish_gps_waypoint()

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config world frame ID ({self.config.world_frame_id})"
            )
            return

        self.robot_pose = Pose2d.from_ros(msg.pose.pose)
        self.advance_waypoint_if_reached()

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config world frame ID ({self.config.world_frame_id})"
            )
            return

        self.grid = WorldOccupancyGrid(msg)

    def state_callback(self, msg: String) -> None:
        self.state = msg.data

    def load_waypoints(self) -> list[Waypoint]:
        with open(self.config.waypoints_file_path) as f:
            data = json.load(f)

        from_ll_client = self.create_client(FromLL, "fromLL")

        self.get_logger().info("Waiting for fromLL service...")
        from_ll_client.wait_for_service()
        self.get_logger().info("fromLL service available, loading waypoints")

        waypoints = []
        for w in data["waypoints"]:
            if "latitude" in w:
                request = FromLL.Request(ll_point=GeoPoint(latitude=w["latitude"], longitude=w["longitude"]))
                future = from_ll_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                point = Point2d.from_ros(future.result().map_point)
            else:
                point = Point2d(x=float(w["x"]), y=float(w["y"]))

            waypoints.append(Waypoint(point=point, no_mans_land=bool(w.get("no_mans_land", False))))

        return waypoints

    def transform_map_to_world(self, point: Point2d) -> Point2d | None:
        try:
            stamped = PointStamped(header=Header(frame_id=self.config.map_frame_id), point=point.to_ros())
            return Point2d.from_ros(self.tf_buffer.transform(stamped, self.config.world_frame_id).point)
        except TransformException as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def publish_gps_waypoint(self) -> None:
        map_point = self.waypoints[self.current_waypoint_index].point
        self.get_logger().info(
            f"Publishing waypoint ({map_point.x:.2f}, {map_point.y:.2f}) in {self.config.map_frame_id} frame"
        )
        self.waypoint_publisher.publish(
            PointStamped(
                header=Header(frame_id=self.config.map_frame_id, stamp=self.get_clock().now().to_msg()),
                point=map_point.to_ros(),
            )
        )

    def advance_waypoint_if_reached(self) -> None:
        if self.robot_pose is None or self.current_waypoint_index >= len(self.waypoints):
            return

        waypoint = self.transform_map_to_world(self.waypoints[self.current_waypoint_index].point)
        if waypoint is None:
            return

        if self.robot_pose.point.distance(waypoint) >= self.config.waypoint_reached_threshold_m:
            return

        if self.waypoints[self.current_waypoint_index].no_mans_land:
            self.in_no_mans_land = not self.in_no_mans_land
            self.get_logger().info(f"{'Entering' if self.in_no_mans_land else 'Exiting'} no man's land")
            self.set_no_mans_land_client.call_async(SetBool.Request(data=self.in_no_mans_land))

        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Final waypoint reached, stopping navigation")
            # We don't call rclpy.shutdown() here because it causes a deadlock in humble
            # https://github.com/ros2/rclpy/issues/1646
            raise SystemExit(1) from None

        self.get_logger().info(f"Waypoint reached, advancing to index {self.current_waypoint_index}")
        self.publish_gps_waypoint()

    def publish_goal(self) -> None:
        if self.robot_pose is None or self.grid is None or self.current_waypoint_index >= len(self.waypoints):
            return

        if self.state == "recovery":
            return

        waypoint = self.transform_map_to_world(self.waypoints[self.current_waypoint_index].point)
        if waypoint is None:
            return

        near_waypoint = self.robot_pose.point.distance(waypoint) < self.config.waypoint_approach_radius_m
        if near_waypoint or self.state == "no_mans_land":
            self.goal_selector.reset()
            self.goal_publisher.publish(
                PointStamped(
                    header=Header(frame_id=self.config.world_frame_id, stamp=self.get_clock().now().to_msg()),
                    point=waypoint.to_ros(),
                )
            )
            return

        if self.config.publish_debug:
            goal, debug = self.goal_selector.step_debug(self.grid, self.robot_pose)
            self.debug_publisher.publish(self.build_debug_markers(debug))
        else:
            goal = self.goal_selector.step(self.grid, self.robot_pose)

        if goal is None:
            self.get_logger().warn("No drivable goal found in occupancy grid")
            self.goal_selector.reset()
            self.set_recovery_client.call_async(SetBool.Request(data=True))
            return

        self.goal_publisher.publish(
            PointStamped(
                header=Header(frame_id=self.config.world_frame_id, stamp=self.get_clock().now().to_msg()),
                point=goal.to_ros(),
            )
        )

    def build_debug_markers(self, debug: GoalSelector.DebugInfo) -> MarkerArray:
        assert self.robot_pose is not None

        def marker(marker_id: int, marker_type: int, **kw) -> Marker:
            # Stamp omitted intentionally to avoid TF extrapolation errors in rviz
            header = Header(frame_id=self.config.world_frame_id)
            return Marker(header=header, ns="goal_selection", id=marker_id, type=marker_type, action=Marker.ADD, **kw)

        scores = [result.score for result in debug.results]
        score_min, score_max = min(scores), max(scores)
        score_span = max(score_max - score_min, 1e-6)

        rays_marker = marker(0, Marker.LINE_LIST, scale=Vector3(x=0.02, y=0.0, z=0.0))
        thick_marker = marker(1, Marker.LINE_LIST, scale=Vector3(x=0.06, y=0.0, z=0.0))

        for result in debug.results:
            end = self.robot_pose.point + Point2d.unit(result.ray.angle) * result.ray.length
            pts = [self.robot_pose.point.to_ros(), end.to_ros()]
            if result is debug.chosen:
                thick_marker.points.extend(pts)
                thick_marker.colors.extend([ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)] * 2)
            elif result is debug.momentum:
                thick_marker.points.extend(pts)
                thick_marker.colors.extend([ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.8)] * 2)
            else:
                t = (result.score - score_min) / score_span
                rays_marker.points.extend(pts)
                rays_marker.colors.extend([ColorRGBA(r=1.0 - t, g=t, b=0.0, a=0.8)] * 2)

        return MarkerArray(markers=[rays_marker, thick_marker])


def main() -> None:
    rclpy.init()
    node = AutonavGoalSelection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
