import math

from geometry_msgs.msg import Twist, Vector3
from rclpy.impl.rcutils_logger import RcutilsLogger
from utils.geometry import Path2d, Pose2d, Rotation2d
from utils.math import clamp

from .config import DifferentialDriveConfig


class DifferentialDriveController:
    def __init__(self, config: DifferentialDriveConfig, logger: RcutilsLogger) -> None:
        self.config = config
        self.logger = logger
        self.path: Path2d | None = None
        self.projection_index: int = 0

    def set_path(self, path: Path2d) -> None:
        self.path = path
        self.projection_index = 0

    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None:
        if self.path is None:
            return None

        if pose.point.distance(self.path[-1]) < self.config.goal_tolerance_m:
            self.logger.info("Reached goal - stopping")
            self.path = None
            return Twist()

        new_projection_index = self.path.project(pose.point, self.projection_index)
        if new_projection_index is None:
            self.logger.warn("DifferentialDrive: no valid projection segment - stopping")
            return Twist()

        if new_projection_index >= len(self.path) - 1:
            self.logger.info("Reached end of path - stopping")
            self.path = None
            return Twist()

        segment_index = int(new_projection_index)
        projected_point = self.path[new_projection_index]

        segment = self.path[segment_index + 1] - self.path[segment_index]
        cross_track_error = segment.cross(projected_point - pose.point) / segment.mag()

        # Adaptive lookahead: ramps up approaching an interior corner and back down leaving it. Stays zero on straight
        # sections, path start/end, and single-segment paths. We use the local segment tangent to avoid oscillation.
        distance_to_corner = (
            (self.path[segment_index + 1] - projected_point).mag() if segment_index < len(self.path) - 2 else math.inf
        )
        distance_past_corner = (projected_point - self.path[segment_index]).mag() if segment_index > 0 else math.inf
        effective_lookahead = max(0.0, self.config.heading_lookahead_m - min(distance_to_corner, distance_past_corner))
        if effective_lookahead > 0.0:
            lookahead_point = self.path.advance(new_projection_index, effective_lookahead)
            heading_error = Rotation2d.from_vector(lookahead_point - pose.point) - pose.rotation
        else:
            heading_error = Rotation2d.from_vector(segment) - pose.rotation

        # cos(heading_error) fades the cross-track term to zero as the robot turns away from the path. Without it, both
        # terms apply at full strength at large heading errors, causing them to fight. With it, heading correction
        # dominates when misaligned and cross-track correction takes over once the robot is aligned.
        angular_velocity = clamp(
            self.config.kp_heading * heading_error.angle + self.config.kp_cross * heading_error.cos * cross_track_error,
            min=-self.config.max_angular_speed_radps,
            max=self.config.max_angular_speed_radps,
        )
        sin_h = abs(heading_error.sin)
        lateral_limit = self.config.max_lateral_speed_mps / sin_h if sin_h > 0 else math.inf
        linear_velocity = max(0.0, min(self.config.target_speed_mps * heading_error.cos, lateral_limit))

        self.projection_index = segment_index
        return Twist(linear=Vector3(x=linear_velocity), angular=Vector3(z=angular_velocity))
