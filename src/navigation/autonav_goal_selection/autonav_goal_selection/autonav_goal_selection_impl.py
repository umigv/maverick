from __future__ import annotations

from dataclasses import dataclass

from utils.geometry import Point2d, Pose2d, Rotation2d
from utils.world_occupancy_grid import WorldOccupancyGrid

from .autonav_goal_selection_config import GoalSelectionParams


class GoalSelector:
    @dataclass(frozen=True)
    class RayCast:
        angle: Rotation2d  # world frame
        length: float

    @dataclass(frozen=True)
    class RayCastResult:
        angle: Rotation2d  # world frame
        length: float
        score: float

    @dataclass(frozen=True)
    class DebugInfo:
        rays: list[GoalSelector.RayCastResult]
        chosen: GoalSelector.RayCastResult | None
        momentum_ray: GoalSelector.RayCastResult

    def __init__(self, params: GoalSelectionParams) -> None:
        self.params = params
        self.momentum_angle: Rotation2d | None = None

    def step(self, grid: WorldOccupancyGrid, robot_pose: Pose2d) -> Point2d | None:
        goal, _ = self.step_debug(grid, robot_pose)
        return goal

    def step_debug(self, grid: WorldOccupancyGrid, robot_pose: Pose2d) -> tuple[Point2d | None, GoalSelector.DebugInfo]:
        casts = self.cast_rays(grid, robot_pose)

        momentum_angle = self.momentum_angle if self.momentum_angle is not None else robot_pose.rotation
        momentum_index = min(range(len(casts)), key=lambda i: abs((casts[i].angle - momentum_angle).angle))

        scores = self.smooth_scores(self.score_rays(casts, casts[momentum_index]))
        rays = [GoalSelector.RayCastResult(w.angle, w.length, s) for w, s in zip(casts, scores, strict=True)]

        chosen = max(rays, key=lambda r: r.score)
        if chosen.length < self.params.min_goal_progress_m:
            return None, GoalSelector.DebugInfo(rays=rays, chosen=None, momentum_ray=rays[momentum_index])

        goal = robot_pose.point + Point2d.unit(chosen.angle) * (chosen.length - self.params.safety_margin_m)
        self.momentum_angle = (
            chosen.angle
            if self.momentum_angle is None
            else self.params.momentum.update_ema(self.momentum_angle, chosen.angle)
        )
        return goal, GoalSelector.DebugInfo(rays=rays, chosen=chosen, momentum_ray=rays[momentum_index])

    def reset(self) -> None:
        self.momentum_angle = None

    def cast_rays(self, grid: WorldOccupancyGrid, robot_pose: Pose2d) -> list[GoalSelector.RayCast]:
        num_rays = int(self.params.arc_angle_rad / self.params.ray_interval_rad) + 1
        start_angle = robot_pose.rotation - Rotation2d(self.params.arc_angle_rad / 2)

        casts: list[GoalSelector.RayCast] = []
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

            casts.append(GoalSelector.RayCast(angle, robot_pose.point.distance(point) - self.params.step_size_m))

        return casts

    def score_rays(self, casts: list[GoalSelector.RayCast], momentum: GoalSelector.RayCast) -> list[float]:
        return [w.length * self.params.momentum.factor(w.angle - momentum.angle, momentum.length) for w in casts]

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
