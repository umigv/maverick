from typing import Any

import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform
import tf2_ros
import utils.config
import utils.lifecycle
import utils.qos
from geometry_msgs.msg import PointStamped, Vector3
from maverick_msgs.msg import MissionState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from utils.geometry import Point2d, Pose2d
from utils.world_occupancy_grid import WorldOccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from .autonav_goal_selection_config import AutonavGoalSelectionConfig
from .autonav_goal_selection_impl import GoalSelector


class AutonavGoalSelection(Node):
    def __init__(self) -> None:
        super().__init__("autonav_goal_selection")

        self.config: AutonavGoalSelectionConfig = utils.config.load(self, AutonavGoalSelectionConfig)

        self.robot_pose: Pose2d | None = None
        self.grid: WorldOccupancyGrid | None = None
        self.mission_state: MissionState | None = None
        self.goal_selector = GoalSelector(self.config.goal_selection_params)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.request_recovery_client = self.create_client(Trigger, "request_recovery")

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(MissionState, "mission_state", self.mission_state_callback, utils.qos.LATCHED)

        self.goal_publisher = self.create_publisher(PointStamped, "goal", 10)
        self.debug_publisher = self.create_publisher(MarkerArray, "goal_selection_debug", 10)

        self.create_timer(self.config.goal_publish_period_s, self.publish_goal)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config world frame ID ({self.config.world_frame_id})"
            )
            return

        self.robot_pose = Pose2d.from_ros(msg.pose.pose)

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config world frame ID ({self.config.world_frame_id})"
            )
            return

        self.grid = WorldOccupancyGrid(msg)

    def mission_state_callback(self, msg: MissionState) -> None:
        self.mission_state = msg

    def transform_to_world(self, stamped: PointStamped) -> Point2d | None:
        # Drop the stamp so tf uses the latest available transform instead of looking up the exact time. Since the state
        # is only updated on change, the timestamp can get stale which would stop us from getting the latest odom error
        # corrections
        stamped = PointStamped(header=Header(frame_id=stamped.header.frame_id), point=stamped.point)

        try:
            return Point2d.from_ros(self.tf_buffer.transform(stamped, self.config.world_frame_id).point)
        except TransformException as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def publish_goal(self) -> None:
        if self.robot_pose is None or self.grid is None or self.mission_state is None:
            return

        if self.mission_state.in_recovery or self.mission_state.mission_complete:
            return

        waypoint = self.transform_to_world(self.mission_state.current_waypoint)
        if waypoint is None:
            return

        distance = self.robot_pose.point.distance(waypoint)
        drive_directly_to_waypoint = (
            distance < self.config.waypoint_approach_radius_m
            or self.mission_state.in_no_mans_land
            or self.mission_state.in_ramp_approach
        )
        if drive_directly_to_waypoint:
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
            self.get_logger().warning("No drivable goal found in occupancy grid")
            self.goal_selector.reset()
            self.request_recovery_client.call_async(Trigger.Request())
            return

        self.goal_publisher.publish(
            PointStamped(
                header=Header(frame_id=self.config.world_frame_id, stamp=self.get_clock().now().to_msg()),
                point=goal.to_ros(),
            )
        )

    def build_debug_markers(self, debug: GoalSelector.DebugInfo) -> MarkerArray:
        assert self.robot_pose is not None

        def marker(marker_id: int, marker_type: int, **kw: Any) -> Marker:
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
    utils.lifecycle.run_node(AutonavGoalSelection)
