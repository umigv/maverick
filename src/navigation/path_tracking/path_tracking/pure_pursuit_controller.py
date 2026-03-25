import math

from geometry_msgs.msg import Twist, Vector3
from rclpy.impl.rcutils_logger import RcutilsLogger
from utils.geometry import Path2d, Point2d, Pose2d
from utils.math import clamp

from .path_tracking_config import PurePursuitConfig


class PurePursuitController:
    def __init__(self, config: PurePursuitConfig, logger: RcutilsLogger) -> None:
        self.config = config
        self.logger = logger
        self.path: Path2d | None = None
        self.lookahead_fractional_index: float = 0.0

    def set_path(self, path: Path2d) -> None:
        self.path = path
        self.lookahead_fractional_index = 0.0

    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None:
        if self.path is None:
            return None

        lookahead_distance = self.compute_lookahead_distance(speed)
        if pose.point.distance(self.path[-1]) < lookahead_distance:
            self.logger.info("Reached goal - stopping")
            self.path = None
            return Twist()

        # fmt: off
        lookahead_point = (self.find_lookahead_point(pose, lookahead_distance) or
                           self.find_projected_lookahead(pose, lookahead_distance))
        # fmt: on
        if lookahead_point is None:
            self.logger.warn("No valid lookahead point found - stopping")
            return Twist()

        local_lookahead = pose.world_to_local(lookahead_point)
        curvature = 2 * local_lookahead.y / local_lookahead.dot(local_lookahead)
        linear = lookahead_distance * self.config.linear_speed_gain
        angular = linear * curvature
        scale = min(1.0, self.config.max_angular_speed_radps / abs(angular)) if angular != 0.0 else 1.0
        return Twist(linear=Vector3(x=linear * scale), angular=Vector3(z=angular * scale))

    def compute_lookahead_distance(self, speed: float) -> float:
        lookahead = self.config.base_lookahead_distance_m + self.config.lookahead_speed_gain * speed
        return clamp(lookahead, min=self.config.min_lookahead_distance_m, max=self.config.max_lookahead_distance_m)

    def find_lookahead_point(self, pose: Pose2d, lookahead_distance: float) -> Point2d | None:
        assert self.path is not None
        robot = pose.point
        start_segment = int(self.lookahead_fractional_index)

        for i in range(start_segment, len(self.path) - 1):
            start = self.path[i]
            end = self.path[i + 1]
            d = end - start
            f = start - robot

            a = d.dot(d)
            b = 2.0 * f.dot(d)
            c = f.dot(f) - lookahead_distance**2

            discriminant = b * b - 4.0 * a * c
            if discriminant < 0.0:
                continue

            t2 = (-b + math.sqrt(discriminant)) / (2.0 * a)
            t1 = (-b - math.sqrt(discriminant)) / (2.0 * a)

            # Check t2 (far intersection) first to prefer the forward-most point.
            for t in (t2, t1):
                if 0.0 <= t <= 1.0 and i + t >= self.lookahead_fractional_index:
                    self.lookahead_fractional_index = i + t
                    return start.lerp(end, t)

        return None

    def find_projected_lookahead(self, pose: Pose2d, lookahead_distance: float) -> Point2d | None:
        assert self.path is not None
        projection_index = self.path.project(pose.point, int(self.lookahead_fractional_index))
        if projection_index is None:
            return None

        self.lookahead_fractional_index = projection_index
        self.logger.warn(f"Lookahead miss, projection fallback aiming at {self.lookahead_fractional_index:.2f}")
        return self.path.advance(projection_index, lookahead_distance)
