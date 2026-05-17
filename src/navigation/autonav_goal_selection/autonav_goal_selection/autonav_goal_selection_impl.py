import math
from dataclasses import dataclass
from enum import Enum, auto

from utils.geometry import Point2d, Pose2d
from utils.world_occupancy_grid import WorldOccupancyGrid

from .autonav_goal_selection_config import GoalSelectionParams


class TerminateReason(Enum):
    OBSTACLE = auto()
    OUT_OF_BOUNDS = auto()  # unknown cell outside the path_planning forward/sideways box


@dataclass(frozen=True)
class RayWalk:
    angle: float  # world-frame heading of the ray
    free_length: float  # distance walked before termination
    terminate_reason: TerminateReason


@dataclass(frozen=True)
class RayResult:
    """A scored ray, returned for goal selection and visualization.

    Attributes:
        walk: The underlying walk (angle, free length, terminate reason).
        score: Combined score used to pick the goal (post-smoothing).
    """

    walk: RayWalk
    score: float


@dataclass(frozen=True)
class GoalSelectionResult:
    """Output of `select_goal`, including debug context for visualization.

    Attributes:
        goal: Chosen goal point in world coordinates, or None if no drivable ray exists.
        rays: One entry per ray cast (ordered from -arc_half_angle to +arc_half_angle).
        chosen_index: Index into `rays` of the chosen ray, or None when goal is None.
        waypoint: The waypoint passed in, included for debug rendering.
    """

    goal: Point2d | None
    rays: list[RayResult]
    chosen_index: int | None
    waypoint: Point2d


def select_goal(
    grid: WorldOccupancyGrid,
    robot_pose: Pose2d,
    waypoint: Point2d,
    params: GoalSelectionParams,
    preferred_angle: float,
) -> GoalSelectionResult:
    """Select a local goal by ray-casting across the robot's forward arc.

    Casts `params.num_rays` rays evenly spaced over [-arc_half_angle, +arc_half_angle] from
    the robot's heading. Each ray is walked in `step_size_m` increments until it hits an
    obstacle or leaves the path_planning unknown-traversal box.

    Each ray is scored as `free_length * ((1 + cos(angle_to_preferred)) / 2) ** alignment_exponent`,
    where `preferred_angle` is either the last chosen ray direction (momentum) or the waypoint
    direction when nearby. Optionally averaged across `neighbor_smoothing_window` neighbors on
    each side. Anti-aligned rays (cos < 0) score 0. The highest-scoring ray wins; if its
    `free_length` is below `min_goal_progress_m`, no goal is returned (treat as stuck).
    """
    walks = _walk_rays(grid, robot_pose, params)
    raw_scores = _score_walks(walks, preferred_angle, params)
    scores = _smooth_scores(raw_scores, params.neighbor_smoothing_window)
    rays = [RayResult(walk=w, score=s) for w, s in zip(walks, scores, strict=True)]

    chosen_index = max(range(len(rays)), key=lambda i: rays[i].score)
    chosen = rays[chosen_index].walk
    if chosen.free_length < params.min_goal_progress_m:
        return GoalSelectionResult(goal=None, rays=rays, chosen_index=None, waypoint=waypoint)

    endpoint_length = max(params.step_size_m, chosen.free_length - params.safety_margin_m)
    direction = Point2d(x=math.cos(chosen.angle), y=math.sin(chosen.angle))
    goal = robot_pose.point + direction * endpoint_length

    return GoalSelectionResult(goal=goal, rays=rays, chosen_index=chosen_index, waypoint=waypoint)


def _walk_rays(grid: WorldOccupancyGrid, robot_pose: Pose2d, params: GoalSelectionParams) -> list[RayWalk]:
    angle_step = (2 * params.arc_half_angle_rad) / (params.num_rays - 1)
    base_angle = robot_pose.rotation.angle - params.arc_half_angle_rad

    walks: list[RayWalk] = []
    for i in range(params.num_rays):
        angle = base_angle + i * angle_step
        direction = Point2d(x=math.cos(angle), y=math.sin(angle))

        free_length = 0.0
        terminate_reason = TerminateReason.OBSTACLE
        step = 0
        while True:
            step += 1
            distance = step * params.step_size_m
            point = robot_pose.point + direction * distance
            state = grid.state(point)

            if state.is_unknown:
                local = robot_pose.world_to_local(point)
                in_box = 0 <= local.x <= params.max_unknown_forward_m and abs(local.y) <= params.max_unknown_sideways_m
                if not in_box:
                    terminate_reason = TerminateReason.OUT_OF_BOUNDS
                    break
            elif not state.is_drivable:
                break

            free_length = distance

        walks.append(RayWalk(angle=angle, free_length=free_length, terminate_reason=terminate_reason))

    return walks


def _score_walks(walks: list[RayWalk], preferred_angle: float, params: GoalSelectionParams) -> list[float]:
    scores: list[float] = []
    for w in walks:
        cos_angle = math.cos(w.angle - preferred_angle)
        raw = max(0.0, (1.0 + cos_angle) / 2.0) ** params.alignment_exponent
        alignment_factor = params.alignment_floor + (1.0 - params.alignment_floor) * raw
        scores.append(w.free_length * alignment_factor)
    return scores


def _smooth_scores(scores: list[float], window: int) -> list[float]:
    if window <= 0:
        return list(scores)

    smoothed: list[float] = []
    for i in range(len(scores)):
        lo = max(0, i - window)
        hi = min(len(scores), i + window + 1)
        smoothed.append(sum(scores[lo:hi]) / (hi - lo))
    return smoothed
