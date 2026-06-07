import json
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import rclpy
import utils.config
import utils.qos
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from maverick_msgs.msg import MissionState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from utils.geometry import Point2d, Pose2d

from .autonav_mission_control_config import AutonavMissionControlConfig


@dataclass(frozen=True)
class Waypoint:
    point: Point2d  # in map frame
    no_mans_land_enter: bool = False  # set in_no_mans_land when this waypoint is reached
    no_mans_land_exit: bool = False  # clear in_no_mans_land when this waypoint is reached
    ramp_approach: bool = False  # activate ramp mode when within ramp_approach_radius_m

    def __post_init__(self) -> None:
        if self.no_mans_land_enter and self.no_mans_land_exit:
            raise ValueError("Waypoint cannot have both no_mans_land_enter and no_mans_land_exit set")


class AutonavMissionControl(Node):
    def __init__(self) -> None:
        super().__init__("autonav_mission_control")

        self.config: AutonavMissionControlConfig = utils.config.load(self, AutonavMissionControlConfig)

        self.robot_pose: Pose2d | None = None
        self.in_recovery: bool = False
        self.in_ramp_approach: bool = False
        self.in_no_mans_land: bool = False
        self.hsv_enabled: bool = True

        self.waypoints: list[Waypoint] = self.load_waypoints()
        self.current_waypoint_index: int = 0

        self.publisher = self.create_publisher(MissionState, "mission_state", utils.qos.LATCHED)

        self.create_service(Trigger, "request_recovery", self.request_recovery_callback)
        self.create_service(Trigger, "recovery_complete", self.recovery_complete_callback)

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.publish_state()

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

            waypoints.append(
                Waypoint(
                    point=point,
                    no_mans_land_enter=bool(w.get("no_mans_land_enter", False)),
                    no_mans_land_exit=bool(w.get("no_mans_land_exit", False)),
                    ramp_approach=bool(w.get("ramp_approach", False)),
                )
            )

        inside = False
        for i, waypoint in enumerate(waypoints):
            if waypoint.no_mans_land_enter:
                if inside:
                    self.get_logger().fatal(
                        f"Waypoint {i}: no_mans_land_enter while already inside no man's land - missing exit before this point"
                    )
                    raise SystemExit(1)
                inside = True
            if waypoint.no_mans_land_exit:
                if not inside:
                    self.get_logger().fatal(
                        f"Waypoint {i}: no_mans_land_exit while not inside no man's land - missing enter before this point"
                    )
                    raise SystemExit(1)
                inside = False
        if inside:
            self.get_logger().fatal("Course file ends inside no man's land - no_mans_land_enter has no matching exit")
            raise SystemExit(1)

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints")
        return waypoints

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.map_frame_id:
            self.get_logger().error(
                f"Odometry frame ({msg.header.frame_id}) does not match map_frame_id ({self.config.map_frame_id})"
            )
            return

        self.robot_pose = Pose2d.from_ros(msg.pose.pose)

        if self.current_waypoint_index >= len(self.waypoints):
            return

        if self.check_waypoint_advancement():
            self.publish_state()

    def check_waypoint_advancement(self) -> bool:
        if self.robot_pose is None:
            return False

        computations: list[tuple[str, Callable[[], tuple[Any, str | None]], Callable[[Any, Any], None] | None]] = [
            ("in_ramp_approach", self.compute_ramp_approach, None),
            ("in_no_mans_land", self.compute_no_mans_land, None),
            ("hsv_enabled", self.compute_enable_hsv, None),
            ("current_waypoint_index", self.compute_waypoint_reached, None),
        ]
        results = [(attr, *compute(), on_change) for attr, compute, on_change in computations]

        changed = False
        for attr, new_val, log_msg, on_change in results:
            if new_val != getattr(self, attr):
                if log_msg:
                    self.get_logger().info(log_msg)
                if on_change is not None:
                    on_change(getattr(self, attr), new_val)
                setattr(self, attr, new_val)
                changed = True

        return changed

    def compute_ramp_approach(self) -> tuple[bool, str | None]:
        assert self.robot_pose is not None
        waypoint = self.waypoints[self.current_waypoint_index]
        distance = self.robot_pose.point.distance(waypoint.point)
        new_val = (
            waypoint.ramp_approach
            and self.config.waypoint_reached_threshold_m < distance <= self.config.ramp_approach_radius_m
        )
        return new_val, "Entering ramp approach" if new_val else "Exiting ramp approach"

    def compute_no_mans_land(self) -> tuple[bool, str | None]:
        assert self.robot_pose is not None
        waypoint = self.waypoints[self.current_waypoint_index]
        distance = self.robot_pose.point.distance(waypoint.point)

        if distance > self.config.waypoint_reached_threshold_m:
            return self.in_no_mans_land, None

        if waypoint.no_mans_land_enter:
            return True, "Entering no man's land"
        elif waypoint.no_mans_land_exit:
            return False, "Exiting no man's land"
        else:
            return self.in_no_mans_land, None

    def compute_waypoint_reached(self) -> tuple[int, str | None]:
        assert self.robot_pose is not None
        distance = self.robot_pose.point.distance(self.waypoints[self.current_waypoint_index].point)

        if distance > self.config.waypoint_reached_threshold_m:
            return self.current_waypoint_index, None

        new_index = self.current_waypoint_index + 1
        if new_index >= len(self.waypoints):
            return new_index, f"Waypoint {self.current_waypoint_index} reached, mission complete"
        else:
            return new_index, f"Waypoint {self.current_waypoint_index} reached, advancing to {new_index}"

    def compute_enable_hsv(self) -> tuple[bool, str | None]:
        assert self.robot_pose is not None
        if not self.in_no_mans_land:
            return True, None

        # Re-enable near the exit waypoint so the robot can see lane markings approaching the zone boundary
        exit_waypoint = next((w for w in self.waypoints[self.current_waypoint_index :] if w.no_mans_land_exit), None)
        if exit_waypoint is not None:
            near_exit = self.robot_pose.point.distance(exit_waypoint.point) <= self.config.hsv_enable_near_exit_radius_m
            return near_exit, "Near no man's land exit, enabling HSV" if near_exit else None

        raise AssertionError("in_no_mans_land but no exit waypoint found - course validation should have caught this")

    def request_recovery_callback(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if not self.in_recovery:
            self.in_recovery = True
            self.get_logger().info("Recovery requested")
            self.publish_state()
        else:
            self.get_logger().info("Recovery already in progress")

        res.success = True
        return res

    def recovery_complete_callback(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self.in_recovery:
            self.in_recovery = False
            self.get_logger().info("Recovery complete")
            self.publish_state()
        else:
            self.get_logger().info("Recovery complete called but not in recovery")

        res.success = True
        return res

    def publish_state(self) -> None:
        self.publisher.publish(
            MissionState(
                in_recovery=self.in_recovery,
                in_ramp_approach=self.in_ramp_approach,
                in_no_mans_land=self.in_no_mans_land,
                hsv_enabled=self.hsv_enabled,
                mission_complete=self.current_waypoint_index >= len(self.waypoints),
                current_waypoint=PointStamped(
                    header=Header(frame_id=self.config.map_frame_id, stamp=self.get_clock().now().to_msg()),
                    point=self.waypoints[min(self.current_waypoint_index, len(self.waypoints) - 1)].point.to_ros(),
                ),
            )
        )


def main() -> None:
    rclpy.init()
    node = AutonavMissionControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
