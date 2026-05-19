import json
import math
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
from .autonav_goal_selection_impl import GoalSelectionResult, TerminateReason, select_goal


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
        self.last_chosen_angle: float | None = None
        self.momentum_cooldown: int = 0
        self.in_no_mans_land: bool = False
        self.in_recovery: bool = False

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
        self.in_recovery = msg.data == "recovery"

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

        if self.robot_pose.point.distance(waypoint) >= self.config.waypoint_reached_threshold:
            return

        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Final waypoint reached, stopping navigation")
            return

        self.get_logger().info(f"Waypoint reached, advancing to index {self.current_waypoint_index}")
        self.publish_gps_waypoint()

        if self.waypoints[self.current_waypoint_index].no_mans_land:
            self.in_no_mans_land = not self.in_no_mans_land
            self.get_logger().info(f"{'Entering' if self.in_no_mans_land else 'Exiting'} no man's land")
            self.set_no_mans_land_client.call_async(SetBool.Request(data=self.in_no_mans_land))

    def publish_goal(self) -> None:
        if self.robot_pose is None or self.grid is None or self.current_waypoint_index >= len(self.waypoints):
            return

        if self.in_recovery:
            return

        waypoint = self.transform_map_to_world(self.waypoints[self.current_waypoint_index].point)
        if waypoint is None:
            return

        dist_to_waypoint = self.robot_pose.point.distance(waypoint)
        if dist_to_waypoint < self.config.near_waypoint_threshold_m:
            waypoint_offset = waypoint - self.robot_pose.point
            preferred_angle = math.atan2(waypoint_offset.y, waypoint_offset.x)
        elif self.last_chosen_angle is not None:
            preferred_angle = self.last_chosen_angle
        else:
            preferred_angle = self.robot_pose.rotation.angle

        result = select_goal(self.grid, self.robot_pose, waypoint, self.config.goal_selection_params, preferred_angle)

        if result.chosen_index is not None:
            if self.momentum_cooldown > 0:
                self.momentum_cooldown -= 1
            else:
                self.last_chosen_angle = result.rays[result.chosen_index].walk.angle
                self.momentum_cooldown = self.config.momentum_cooldown_frames

        if self.config.publish_debug:
            self.debug_publisher.publish(self._build_debug_markers(result))

        if result.goal is None:
            self.get_logger().warn("No drivable goal found in occupancy grid")
            self.set_recovery_client.call_async(SetBool.Request(data=True))
            return

        self.get_logger().info(
            f"Publishing local goal ({result.goal.x:.2f}, {result.goal.y:.2f}) in {self.config.world_frame_id} frame"
        )
        self.goal_publisher.publish(
            PointStamped(
                header=Header(frame_id=self.config.world_frame_id, stamp=self.get_clock().now().to_msg()),
                point=result.goal.to_ros(),
            )
        )

    def _build_debug_markers(self, result: GoalSelectionResult) -> MarkerArray:
        # robot_pose is non-None here because publish_goal early-returns otherwise
        assert self.robot_pose is not None
        header = Header(frame_id=self.config.world_frame_id, stamp=self.get_clock().now().to_msg())
        ns = "goal_selection"

        scores = [r.score for r in result.rays]
        score_min, score_max = min(scores), max(scores)
        score_span = max(score_max - score_min, 1e-6)

        rays_marker = Marker(
            header=header,
            ns=ns,
            id=0,
            type=Marker.LINE_LIST,
            action=Marker.ADD,
            scale=Vector3(x=0.02, y=0.0, z=0.0),
        )
        for ray in result.rays:
            start = self.robot_pose.point
            end = start + Point2d(x=math.cos(ray.walk.angle), y=math.sin(ray.walk.angle)) * ray.walk.free_length
            rays_marker.points.extend([start.to_ros(), end.to_ros()])
            color = _score_to_color((ray.score - score_min) / score_span)
            rays_marker.colors.extend([color, color])

        terminator_points = []
        terminator_colors = []
        for ray in result.rays:
            end = (
                self.robot_pose.point
                + Point2d(x=math.cos(ray.walk.angle), y=math.sin(ray.walk.angle)) * ray.walk.free_length
            )
            terminator_points.append(end.to_ros())
            terminator_colors.append(_terminate_color(ray.walk.terminate_reason))
        terminators_marker = Marker(
            header=header,
            ns=ns,
            id=1,
            type=Marker.SPHERE_LIST,
            action=Marker.ADD,
            scale=Vector3(x=0.06, y=0.06, z=0.06),
            points=terminator_points,
            colors=terminator_colors,
        )

        markers = [rays_marker, terminators_marker]

        if result.chosen_index is not None and result.goal is not None:
            chosen = result.rays[result.chosen_index]
            chosen_end = (
                self.robot_pose.point
                + Point2d(x=math.cos(chosen.walk.angle), y=math.sin(chosen.walk.angle)) * chosen.walk.free_length
            )
            chosen_ray_marker = Marker(
                header=header,
                ns=ns,
                id=2,
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                scale=Vector3(x=0.06, y=0.0, z=0.0),
                color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                points=[self.robot_pose.point.to_ros(), chosen_end.to_ros()],
            )
            goal_marker = Marker(
                header=header,
                ns=ns,
                id=3,
                type=Marker.SPHERE,
                action=Marker.ADD,
                scale=Vector3(x=0.2, y=0.2, z=0.2),
                color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                pose=Pose2d(point=result.goal, rotation=self.robot_pose.rotation).to_ros(),
            )
            markers.extend([chosen_ray_marker, goal_marker])

        waypoint_marker = Marker(
            header=header,
            ns=ns,
            id=4,
            type=Marker.ARROW,
            action=Marker.ADD,
            scale=Vector3(x=0.05, y=0.1, z=0.1),
            color=ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0),
            points=[self.robot_pose.point.to_ros(), result.waypoint.to_ros()],
        )
        markers.append(waypoint_marker)

        return MarkerArray(markers=markers)


def _score_to_color(t: float) -> ColorRGBA:
    """Map normalized score t in [0, 1] to red->yellow->green."""
    t = max(0.0, min(1.0, t))
    return ColorRGBA(r=1.0 - t, g=t, b=0.0, a=0.8)


def _terminate_color(reason: TerminateReason) -> ColorRGBA:
    if reason is TerminateReason.OBSTACLE:
        return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # red
    return ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # yellow — out of bounds


def main() -> None:
    rclpy.init()
    node = AutonavGoalSelection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
