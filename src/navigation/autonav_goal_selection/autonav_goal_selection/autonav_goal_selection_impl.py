from dataclasses import dataclass

from utils.geometry import Point2d, Pose2d, Rotation2d
from utils.world_occupancy_grid import WorldOccupancyGrid

from .autonav_goal_selection_config import GoalSelectionParams


class GoalSelector:
    @dataclass(frozen=True)
    class Ray:
        angle: Rotation2d  # world frame
        length: float

    @dataclass(frozen=True)
    class RayResult:
        ray: GoalSelector.Ray
        score: float

    @dataclass(frozen=True)
    class DebugInfo:
        results: list[GoalSelector.RayResult]
        chosen: GoalSelector.RayResult | None
        momentum: GoalSelector.RayResult

    def __init__(self, params: GoalSelectionParams) -> None:
        self.params = params
        self.momentum_angle: Rotation2d | None = None

    def step(self, grid: WorldOccupancyGrid, robot_pose: Pose2d) -> Point2d | None:
        goal, _ = self.step_debug(grid, robot_pose)
        return goal

    def step_debug(self, grid: WorldOccupancyGrid, robot_pose: Pose2d) -> tuple[Point2d | None, GoalSelector.DebugInfo]:
        rays = self.cast_rays(grid, robot_pose)

        momentum_angle = self.momentum_angle if self.momentum_angle is not None else robot_pose.rotation
        momentum_index = min(range(len(rays)), key=lambda i: abs((rays[i].angle - momentum_angle).angle))

        scores = self.smooth_scores(self.score_rays(rays, rays[momentum_index]))
        results = [GoalSelector.RayResult(w, s) for w, s in zip(rays, scores, strict=True)]

        chosen = max(results, key=lambda r: r.score)
        if chosen.ray.length < self.params.min_goal_progress_m:
            return None, GoalSelector.DebugInfo(results=results, chosen=None, momentum=results[momentum_index])

        goal = robot_pose.point + Point2d.unit(chosen.ray.angle) * (chosen.ray.length - self.params.safety_margin_m)
        self.momentum_angle = (
            chosen.ray.angle
            if self.momentum_angle is None
            else self.params.momentum.update_ema(self.momentum_angle, chosen.ray.angle)
        )
        return goal, GoalSelector.DebugInfo(results=results, chosen=chosen, momentum=results[momentum_index])

    def reset(self) -> None:
        self.momentum_angle = None

    def cast_rays(self, grid: WorldOccupancyGrid, robot_pose: Pose2d) -> list[GoalSelector.Ray]:
        num_rays = int(self.params.arc_angle_rad / self.params.ray_interval_rad) + 1
        start_angle = robot_pose.rotation - Rotation2d(self.params.arc_angle_rad / 2)

        rays: list[GoalSelector.Ray] = []
        for i in range(num_rays):
            angle = start_angle + Rotation2d(i * self.params.ray_interval_rad)
            step_vector = Point2d.unit(angle) * self.params.step_size_m
            point = robot_pose.point

            while True:
                point = point + step_vector
                state = grid.state(point)

                if state.is_unknown:
                    local = robot_pose.world_to_local(point)
                    within_unknown_limits = (
                        0 <= local.x <= self.params.max_unknown_forward_m
                        and abs(local.y) <= self.params.max_unknown_sideways_m
                    )
                    if not within_unknown_limits:
                        break
                elif not state.is_drivable:
                    break

            rays.append(GoalSelector.Ray(angle, robot_pose.point.distance(point) - self.params.step_size_m))

        return rays

    def score_rays(self, rays: list[GoalSelector.Ray], momentum: GoalSelector.Ray) -> list[float]:
        return [w.length * self.params.momentum.factor(w.angle - momentum.angle, momentum.length) for w in rays]

    def smooth_scores(self, scores: list[float]) -> list[float]:
        window = self.params.neighbor_smoothing_window
        if window <= 0:
            return list(scores)

        smoothed: list[float] = []
        for i in range(len(scores)):
            lo = max(0, i - window)
            hi = min(len(scores), i + window + 1)
            smoothed.append(sum(scores[lo:hi]) / (hi - lo))
        return smoothed
