import math

from geometry_msgs.msg import Twist, Vector3
from rclpy.impl.rcutils_logger import RcutilsLogger
from utils.geometry import Path2d, Point2d, Pose2d, Rotation2d
from utils.math import clamp

from .path_tracking_config import StanleyConfig


class StanleyController:
    def __init__(self, config: StanleyConfig, logger: RcutilsLogger) -> None:
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

        front = pose.local_to_world(Point2d(self.config.front_offset_m, 0.0))
        if front.distance(self.path[-1]) < self.config.goal_tolerance_m:
            self.logger.info("Reached goal - stopping")
            self.path = None
            return Twist()

        new_projection_index = self.path.project(front, self.projection_index)
        if new_projection_index is None:
            self.logger.warn("Stanley: no valid projection segment - stopping")
            return Twist()
        elif new_projection_index >= len(self.path) - 1:
            self.logger.info("Reached end of path - stopping")
            self.path = None
            return Twist()

        segment_index = int(new_projection_index)
        projected_point = self.path[new_projection_index]

        segment = self.path[segment_index + 1] - self.path[segment_index]
        heading_error = (Rotation2d.from_vector(segment) - pose.rotation).angle
        cross_track_error = segment.cross(projected_point - front) / segment.mag()

        linear_velocity = self.compute_speed_limit(new_projection_index)

        steering_angle = clamp(
            heading_error + math.atan2(self.config.cross_track_gain * cross_track_error, max(speed, 0.1)),
            min=-self.config.max_steer_rad,
            max=self.config.max_steer_rad,
        )

        # Bicycle model: ω = v·tan(δ)/L, using front_offset_m as wheelbase (unicycle approximation for diff drive)
        angular_velocity = clamp(
            linear_velocity * math.tan(steering_angle) / self.config.front_offset_m,
            min=-self.config.max_angular_speed_radps,
            max=self.config.max_angular_speed_radps,
        )

        # We start searching from the segment containing the last projection
        self.projection_index = segment_index
        return Twist(linear=Vector3(x=linear_velocity), angular=Vector3(z=angular_velocity))

    def compute_speed_limit(self, index: float) -> float:
        """Cap speed by the lateral-accel budget: v^2 / r <= a_lat, with r = arclength / heading_change over the
        lookahead horizon. Slows the robot *before* it reaches an upcoming corner.
        """
        total_heading_change, arclength = self.compute_heading_change(index)
        if total_heading_change <= 1e-3:
            return self.config.target_speed_mps

        max_speed_mps = math.sqrt(self.config.max_lateral_accel_mps2 * arclength / total_heading_change)
        return min(self.config.target_speed_mps, max_speed_mps)

    def compute_heading_change(self, index: float) -> tuple[float, float]:
        """Accumulate absolute path heading change and arclength ahead of `index`.

        Walks forward from the projection until either `curvature_lookahead_m` of arclength has been consumed or the
        path ends. Heading change accrues at each vertex crossed.
        """
        assert self.path is not None
        segment_index = int(index)
        point = self.path[index]

        remaining = self.config.curvature_lookahead_m
        heading_change = 0.0
        arclength = 0.0

        def advance(distance: float) -> None:
            nonlocal remaining, arclength
            consumed = min(distance, remaining)
            arclength += consumed
            remaining -= consumed

        prev_segment = self.path[segment_index + 1] - point
        advance(prev_segment.mag())
        i = segment_index + 1
        while i < len(self.path) - 1 and remaining > 0.0:
            next_segment = self.path[i + 1] - self.path[i]
            heading_change += abs((Rotation2d.from_vector(next_segment) - Rotation2d.from_vector(prev_segment)).angle)
            advance(next_segment.mag())
            prev_segment = next_segment
            i += 1

        return heading_change, arclength
