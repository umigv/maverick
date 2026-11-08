"""Testing data collection for path tracking."""

import csv
import math
from collections.abc import Callable
from dataclasses import astuple, dataclass, fields
from datetime import datetime
from pathlib import Path
from typing import IO

from geometry_msgs.msg import Twist
from rclpy.impl.rcutils_logger import RcutilsLogger
from utils.geometry import Path2d, Pose2d, Rotation2d


@dataclass(frozen=True)
class WaypointRow:
    """Waypoint data."""

    time_seconds: float
    path_id: int
    waypoint_index: int
    waypoint_x: float
    waypoint_y: float
    robot_x: float
    robot_y: float
    error_m: float

    @classmethod
    def columns(cls) -> list[str]:
        return [f.name for f in fields(cls)]

    def to_row(self) -> list[object]:
        return list(astuple(self))


@dataclass(frozen=True)
class HeadingRow:
    """Heading data."""

    time_seconds: float
    path_id: int
    robot_x: float
    robot_y: float
    robot_heading_deg: float
    path_heading_deg: float
    commanded_heading_deg: float
    path_heading_error_deg: float
    commanded_heading_error_deg: float
    cross_track_error_m: float
    linear_mps: float
    angular_radps: float

    @classmethod
    def columns(cls) -> list[str]:
        return [f.name for f in fields(cls)]

    def to_row(self) -> list[object]:
        return list(astuple(self))


class PathTrackingMetrics:
    def __init__(self, logger: RcutilsLogger, output_dir: Path | None) -> None:
        """Create a metrics recorder.

        Args:
            logger: Node logger for command line output.
            output_dir: Directory for CSV files. None disables CSV output.
        """
        self.logger = logger
        self.output_dir = output_dir.expanduser() if output_dir is not None else None

        self.path: Path2d | None = None
        self.path_id: int = 0
        self.start_time_seconds: float | None = None

        self.next_waypoint: int = 1
        self.closest_distance: float = math.inf
        self.closest_pose: Pose2d | None = None

        self.files: list[IO[str]] = []
        self.write_waypoint_row: Callable[[list[object]], object] | None = None
        self.write_heading_row: Callable[[list[object]], object] | None = None

    def on_path(self, path: Path2d, time_seconds: float) -> None:
        """Call when a new path is given to the controller."""
        if self.start_time_seconds is None:
            self.start_run(time_seconds)

        self.path = path
        self.path_id += 1
        self.next_waypoint = 1
        self.closest_distance = math.inf
        self.closest_pose = None

    def on_control(self, pose: Pose2d, target_heading: Rotation2d | None, cmd: Twist, time_seconds: float) -> None:
        """Call each control tick that produced a command while following a path."""
        if self.path is None or self.start_time_seconds is None:
            return

        path = self.path
        projection = path.project(pose.point, max(0, self.next_waypoint - 1))
        if projection is None:
            return

        self.update_waypoints(pose, projection, time_seconds)

        segment_index = min(int(projection), len(path) - 2)
        segment = path[segment_index + 1] - path[segment_index]
        path_heading = Rotation2d.from_vector(segment)
        cross_track_error = segment.cross(pose.point - path[projection]) / segment.mag()

        if self.write_heading_row is not None:
            row = HeadingRow(
                time_seconds=time_seconds - self.start_time_seconds,
                path_id=self.path_id,
                robot_x=pose.point.x,
                robot_y=pose.point.y,
                robot_heading_deg=math.degrees(pose.rotation.angle),
                path_heading_deg=math.degrees(path_heading.angle),
                commanded_heading_deg=math.degrees(target_heading.angle) if target_heading is not None else math.nan,
                path_heading_error_deg=math.degrees(abs((pose.rotation - path_heading).angle)),
                commanded_heading_error_deg=(
                    math.degrees(abs((pose.rotation - target_heading).angle))
                    if target_heading is not None
                    else math.nan
                ),
                cross_track_error_m=cross_track_error,
                linear_mps=cmd.linear.x,
                angular_radps=cmd.angular.z,
            )
            self.write_heading_row(row.to_row())

    def update_waypoints(self, pose: Pose2d, projection: float, time_s: float) -> None:
        """Track the closest approach to the next waypoint and record it once the robot passes it."""
        assert self.path is not None
        last_index = len(self.path) - 1

        while self.next_waypoint <= last_index:
            distance = pose.point.distance(self.path[self.next_waypoint])
            if distance < self.closest_distance:
                self.closest_distance = distance
                self.closest_pose = pose

            # The final waypoint is recorded by on_goal_reached, since the projection never moves past it.
            if self.next_waypoint == last_index or projection < self.next_waypoint:
                return

            self.record_waypoint(time_s)

    def record_waypoint(self, time_seconds: float) -> None:
        assert self.path is not None and self.start_time_seconds is not None and self.closest_pose is not None
        waypoint = self.path[self.next_waypoint]
        if self.write_waypoint_row is not None:
            row = WaypointRow(
                time_seconds=time_seconds - self.start_time_seconds,
                path_id=self.path_id,
                waypoint_index=self.next_waypoint,
                waypoint_x=waypoint.x,
                waypoint_y=waypoint.y,
                robot_x=self.closest_pose.point.x,
                robot_y=self.closest_pose.point.y,
                error_m=self.closest_distance,
            )
            self.write_waypoint_row(row.to_row())
        self.next_waypoint += 1
        self.closest_distance = math.inf
        self.closest_pose = None

    def on_goal_reached(self, pose: Pose2d, time_seconds: float) -> None:
        """Call once when the robot reaches the final goal. Logs the total time and ends the run."""
        if self.start_time_seconds is None:
            return

        if self.path is not None:
            last_index = len(self.path) - 1
            # Record any remaining waypoints, including the final goal itself.
            while self.next_waypoint <= last_index:
                distance = pose.point.distance(self.path[self.next_waypoint])
                if distance < self.closest_distance:
                    self.closest_distance = distance
                    self.closest_pose = pose
                self.record_waypoint(time_seconds)

        elapsed = time_seconds - self.start_time_seconds
        self.logger.info(f"[metrics] Reached final goal in {elapsed:.2f} s")
        self.end_run()

    def start_run(self, time_seconds: float) -> None:
        self.start_time_seconds = time_seconds
        self.path_id = 0
        if self.output_dir is None:
            return

        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
            waypoint_path = self.output_dir / f"{stamp}_waypoints.csv"
            heading_path = self.output_dir / f"{stamp}_heading.csv"
            waypoint_file = waypoint_path.open("w", newline="", buffering=1)
            heading_file = heading_path.open("w", newline="", buffering=1)
        except OSError as e:
            self.logger.error(f"[metrics] Could not open CSV files in {self.output_dir}: {e}")
            return

        self.files = [waypoint_file, heading_file]
        self.write_waypoint_row = csv.writer(waypoint_file).writerow
        self.write_heading_row = csv.writer(heading_file).writerow
        self.write_waypoint_row(WaypointRow.columns())
        self.write_heading_row(HeadingRow.columns())
        self.logger.info(f"[metrics] Recording to {waypoint_path} and {heading_path}")

    def end_run(self) -> None:
        for f in self.files:
            f.close()
        self.files = []
        self.write_waypoint_row = None
        self.write_heading_row = None
        self.start_time_seconds = None
        self.path = None
