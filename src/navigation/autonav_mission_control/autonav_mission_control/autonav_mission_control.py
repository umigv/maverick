import json
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, NamedTuple

import rclpy
import utils.config
import utils.lifecycle
import utils.qos
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from maverick_msgs.msg import MissionState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.timer import Timer
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


class StateUpdate(NamedTuple):
    attr: str  # name of the state attribute on AutonavMissionControl
    compute: Callable[[], tuple[Any, str | None]]  # returns (new value, optional log message)
    on_change: Callable[[Any, Any], None] | None = None  # called with (old value, new value)


class AutonavMissionControl(Node):
    def __init__(self) -> None:
        super().__init__("autonav_mission_control")

        self.config: AutonavMissionControlConfig = utils.config.load(self, AutonavMissionControlConfig)

        self.robot_pose: Pose2d | None = None
        self.in_recovery: bool = False
        self.in_ramp_approach: bool = False
        self.in_no_mans_land: bool = False
        self.lane_detection_enabled: bool = True
        self.mission_complete: bool = False
        self.mission_complete_timer: Timer | None = None
        self.mission_complete_goal: Point2d | None = None

        self.waypoints: list[Waypoint] = self.load_waypoints()
        self.current_waypoint_index: int = 0

        self.publisher = self.create_publisher(MissionState, "mission_state", utils.qos.LATCHED)

        self.create_service(Trigger, "request_recovery", self.request_recovery_callback)
        self.create_service(Trigger, "recovery_complete", self.recovery_complete_callback)

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.publish_state()

    def load_waypoints(self) -> list[Waypoint]:
        with self.config.waypoints_file_path.open() as f:
            data = json.load(f)

        if "waypoints" not in data:
            self.get_logger().fatal(f"{self.config.waypoints_file_path} has no waypoints key")
            raise SystemExit(1)

        if len(data["waypoints"]) == 0:
            self.get_logger().fatal(f"{self.config.waypoints_file_path} has an empty waypoints list")
            raise SystemExit(1)

        # Only the lat/lon waypoints need fromLL service conversion; waypoints given as x/y don't need it.
        # Skip creating/waiting on the service entirely when no waypoint requires it.
        from_ll_client = self.create_client(FromLL, "fromLL")
        if any("latitude" in w for w in data["waypoints"]):
            self.get_logger().info("Waiting for fromLL service...")
            timeout = self.config.from_ll_service_timeout_s
            if not from_ll_client.wait_for_service(timeout_sec=timeout):
                self.get_logger().fatal(
                    f"fromLL service not available after {timeout:g}s; cannot convert lat/lon waypoints"
                )
                raise SystemExit(1)
            self.get_logger().info("fromLL service available, loading waypoints")

        waypoints = []
        for w in data["waypoints"]:
            if "latitude" in w:
                request = FromLL.Request(ll_point=GeoPoint(latitude=w["latitude"], longitude=w["longitude"]))
                future = from_ll_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response is None:
                    self.get_logger().fatal("fromLL service call returned no response")
                    raise SystemExit(1)
                point = Point2d.from_ros(response.map_point)
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

        in_no_mans_land = False
        for i, waypoint in enumerate(waypoints):
            if waypoint.no_mans_land_enter:
                if in_no_mans_land:
                    self.get_logger().fatal(
                        f"Waypoint {i}: no_mans_land_enter while already inside no man's land (missing exit before this point)"
                    )
                    raise SystemExit(1)
                in_no_mans_land = True
            if waypoint.no_mans_land_exit:
                if not in_no_mans_land:
                    self.get_logger().fatal(
                        f"Waypoint {i}: no_mans_land_exit while not inside no man's land (missing enter before this point)"
                    )
                    raise SystemExit(1)
                in_no_mans_land = False
        if in_no_mans_land:
            self.get_logger().fatal("Course file ends inside no man's land (no_mans_land_enter has no matching exit)")
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

        if self.update_mission_state():
            self.publish_state()

    def update_mission_state(self) -> bool:
        if self.robot_pose is None:
            return False

        updates = [
            StateUpdate("in_ramp_approach", self.compute_ramp_approach),
            StateUpdate("in_no_mans_land", self.compute_no_mans_land),
            StateUpdate("lane_detection_enabled", self.compute_enable_lane_detection),
            StateUpdate(
                "current_waypoint_index", self.compute_waypoint_reached, on_change=self.on_waypoint_index_change
            ),
        ]
        # Run all compute functions first so each one sees the pre-update state, then apply:
        # for each attr (e.g. self.in_ramp_approach), getattr reads the stored value to diff
        # against the computed one, and on a change we log, fire on_change, and setattr the new value.
        results = [(u.attr, *u.compute(), u.on_change) for u in updates]

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

        if waypoint.no_mans_land_exit:
            return False, "Exiting no man's land"
        return self.in_no_mans_land, None

    def compute_waypoint_reached(self) -> tuple[int, str | None]:
        assert self.robot_pose is not None
        distance = self.robot_pose.point.distance(self.waypoints[self.current_waypoint_index].point)

        if distance > self.config.waypoint_reached_threshold_m:
            return self.current_waypoint_index, None

        new_index = self.current_waypoint_index + 1
        if new_index >= len(self.waypoints):
            return new_index, f"Waypoint {self.current_waypoint_index} reached, final waypoint"
        return new_index, f"Waypoint {self.current_waypoint_index} reached, advancing to {new_index}"

    def on_waypoint_index_change(self, _old: int, new: int) -> None:
        if new >= len(self.waypoints) and self.mission_complete_timer is None:
            assert self.robot_pose is not None
            # ensure the goal is far enough that it won't be reached during normal operation
            self.mission_complete_goal = self.robot_pose.local_to_world(Point2d(x=10000.0, y=0.0))
            self.mission_complete_timer = self.create_timer(self.config.mission_complete_delay_s, self.complete_mission)

    def complete_mission(self) -> None:
        assert self.mission_complete_timer is not None
        self.mission_complete_timer.cancel()  # rclpy timers repeat; cancelling makes this one-shot

        if not self.mission_complete:
            self.mission_complete = True
            self.get_logger().info("Mission complete")
            self.publish_state()

            # Exiting signals mission completion to the launch, which shuts down autonav
            self.get_logger().info("Shutting down")
            raise SystemExit(0)

    def compute_enable_lane_detection(self) -> tuple[bool, str | None]:
        assert self.robot_pose is not None
        if not self.in_no_mans_land:
            return True, None

        # Re-enable near the exit waypoint so the robot can see lane markings approaching the zone boundary
        exit_waypoint = next((w for w in self.waypoints[self.current_waypoint_index :] if w.no_mans_land_exit), None)
        if exit_waypoint is not None:
            near_exit = (
                self.robot_pose.point.distance(exit_waypoint.point)
                <= self.config.lane_detection_enable_near_exit_radius_m
            )
            return near_exit, "Near no man's land exit, enabling lane detection" if near_exit else None

        raise AssertionError("in_no_mans_land but no exit waypoint found (broken course validation?)")

    def request_recovery_callback(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if not self.in_recovery:
            self.in_recovery = True
            self.get_logger().info("Recovery requested")
            self.publish_state()
        else:
            self.get_logger().warning("Recovery already in progress")

        res.success = True
        return res

    def recovery_complete_callback(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self.in_recovery:
            self.in_recovery = False
            self.get_logger().info("Recovery complete")
            self.publish_state()
        else:
            self.get_logger().warning("Recovery complete called but not in recovery")

        res.success = True
        return res

    def publish_state(self) -> None:
        if self.current_waypoint_index < len(self.waypoints):
            goal = self.waypoints[self.current_waypoint_index].point
        else:
            assert self.mission_complete_goal is not None
            goal = self.mission_complete_goal

        self.publisher.publish(
            MissionState(
                in_recovery=self.in_recovery,
                in_ramp_approach=self.in_ramp_approach,
                in_no_mans_land=self.in_no_mans_land,
                lane_detection_enabled=self.lane_detection_enabled,
                mission_complete=self.mission_complete,
                current_waypoint=PointStamped(
                    header=Header(frame_id=self.config.map_frame_id, stamp=self.get_clock().now().to_msg()),
                    point=goal.to_ros(),
                ),
            )
        )


def main() -> None:
    utils.lifecycle.run_node(AutonavMissionControl)
