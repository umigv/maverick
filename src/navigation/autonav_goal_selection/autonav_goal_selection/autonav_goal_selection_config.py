import math
from dataclasses import dataclass

from utils.geometry import Rotation2d


@dataclass(frozen=True)
class MomentumParams:
    """Controls all momentum behavior in the ray-cast goal selection.

    Attributes:
        alignment_gain: Sharpness of the penalty for rays misaligned with the momentum ray. 0 disables the bias; higher
            values penalize deviation more aggressively.
        alignment_floor: Minimum alignment factor regardless of angle. Higher values mean even anti-aligned rays retain
            most of their score.
        obstacle_threshold_m: Free length of the momentum-aligned ray below which momentum weight is reduced. Above this
            threshold momentum is fully applied; below it the weight scales down so longer escape routes can win.
        obstacle_gain: Exponent controlling the shape of the momentum scaling curve.
            scale = (length / threshold) ** gain, clamped to [0, 1].
            gain < 1: weight drops quickly even for moderately short rays.
            gain = 1: linear drop.
            gain > 1: weight stays near 1 until very close to 0, then drops sharply.
        ema_alpha: EMA smoothing factor for the momentum angle. Each frame the stored momentum drifts alpha *
            angular_diff toward the chosen ray. Lower values make momentum change more slowly (stable in open space);
            higher values react faster to obstacles.
            Typical range: 0.05-0.3.
    """

    alignment_gain: float
    alignment_floor: float
    obstacle_threshold_m: float
    obstacle_gain: float
    ema_alpha: float

    def __post_init__(self) -> None:
        if self.alignment_gain < 0:
            raise ValueError("MomentumParams: alignment_gain must be >= 0")
        if not (0.0 <= self.alignment_floor < 1.0):
            raise ValueError("MomentumParams: alignment_floor must be in [0, 1)")
        if self.obstacle_threshold_m <= 0:
            raise ValueError("MomentumParams: obstacle_threshold_m must be > 0")
        if self.obstacle_gain <= 0:
            raise ValueError("MomentumParams: obstacle_gain must be > 0")
        if not (0.0 < self.ema_alpha <= 1.0):
            raise ValueError("MomentumParams: ema_alpha must be in (0, 1]")

    def factor(self, angle_diff: Rotation2d, momentum_ray_length: float) -> float:
        """Alignment factor for a ray at angle deviation `d` from momentum."""
        clearance = min(1.0, momentum_ray_length / self.obstacle_threshold_m)
        obstacle_scale: float = clearance**self.obstacle_gain
        raw_alignment: float = ((1.0 + angle_diff.cos) / 2.0) ** self.alignment_gain
        alignment = self.alignment_floor + (1.0 - self.alignment_floor) * raw_alignment
        return 1.0 - obstacle_scale + obstacle_scale * alignment

    def update_ema(self, current: Rotation2d, target: Rotation2d) -> Rotation2d:
        return current + (target - current) * self.ema_alpha


@dataclass(frozen=True)
class GoalSelectionParams:
    """Parameters controlling the ray-cast goal selection heuristic.

    Attributes:
        momentum: All parameters controlling momentum strength, alignment bias, obstacle scaling, and EMA smoothing.
        arc_angle_rad: Full angular width (rad) of the forward arc; rays span symmetrically around the robot's heading.
            math.pi = 180° forward semicircle.
        ray_interval_rad: Angular spacing (rad) between adjacent rays. Controls density independently of arc size;
            num_rays is derived as int(arc / interval) + 1.
        step_size_m: Step size used when walking each ray; should be ~grid resolution.
        min_goal_progress_m: Minimum length the chosen ray must have for a goal to be published. If the highest-scoring
            ray's length is below this, `select_goal` returns no goal (caller should treat as "stuck"). Set to 0 to
            always publish.
        safety_margin_m: Distance (m) the chosen endpoint is pulled back from where the ray terminated.
        neighbor_smoothing_window: Number of neighbors on each side averaged into each ray's score before picking. 0
            disables smoothing.
        max_unknown_forward_m: Mirror of path_planning: how far forward unknown cells are considered drivable when
            walking a ray.
        max_unknown_sideways_m: Mirror of path_planning: how far sideways unknown cells are considered drivable when
            walking a ray.
    """

    momentum: MomentumParams = MomentumParams(
        alignment_gain=2.0,
        alignment_floor=0.1,
        obstacle_threshold_m=4.0,
        obstacle_gain=2.0,
        ema_alpha=0.1,
    )
    arc_angle_rad: float = math.pi
    ray_interval_rad: float = math.radians(1.25)
    step_size_m: float = 0.05
    min_goal_progress_m: float = 0.9
    safety_margin_m: float = 0.9
    neighbor_smoothing_window: int = 2
    max_unknown_forward_m: float = 5.0
    max_unknown_sideways_m: float = 2.5

    def __post_init__(self) -> None:
        if not (0 < self.arc_angle_rad <= 2 * math.pi):
            raise ValueError("GoalSelectionParams: arc_angle_rad must be in (0, 2*pi]")
        if not (0 < self.ray_interval_rad <= self.arc_angle_rad):
            raise ValueError("GoalSelectionParams: ray_interval_rad must be in (0, arc_angle_rad]")
        if self.step_size_m <= 0:
            raise ValueError("GoalSelectionParams: step_size_m must be > 0")
        if self.min_goal_progress_m < 0:
            raise ValueError("GoalSelectionParams: min_goal_progress_m must be >= 0")
        if self.safety_margin_m < 0:
            raise ValueError("GoalSelectionParams: safety_margin_m must be >= 0")
        if self.min_goal_progress_m < self.safety_margin_m:
            raise ValueError("GoalSelectionParams: min_goal_progress_m must be >= safety_margin_m")
        if self.neighbor_smoothing_window < 0:
            raise ValueError("GoalSelectionParams: neighbor_smoothing_window must be >= 0")
        if self.max_unknown_forward_m < 0:
            raise ValueError("GoalSelectionParams: max_unknown_forward_m must be >= 0")
        if self.max_unknown_sideways_m < 0:
            raise ValueError("GoalSelectionParams: max_unknown_sideways_m must be >= 0")


@dataclass(frozen=True)
class AutonavGoalSelectionConfig:
    """Configuration for autonomous navigation goal selection.

    Attributes:
        goal_selection_params: Parameters for the ray-cast goal selection algorithm.
        goal_publish_period_s: How often (seconds) to publish a new local goal.
        waypoint_approach_radius_m: Distance (m) from the current waypoint within which ray-cast goal selection is
            bypassed and the waypoint itself is published directly as the goal.
        world_frame_id: TF frame ID for the world coordinate frame.
        publish_debug: When true, publish a MarkerArray on `goal_selection_debug` showing all rays, the chosen ray, the
            chosen endpoint, and the waypoint direction.
    """

    goal_selection_params: GoalSelectionParams
    goal_publish_period_s: float = 1
    waypoint_approach_radius_m: float = 5.0
    world_frame_id: str = "odom"
    publish_debug: bool = True

    def __post_init__(self) -> None:
        if self.goal_publish_period_s <= 0:
            raise ValueError("AutonavGoalSelectionConfig: goal_publish_period_s must be > 0")
        if self.waypoint_approach_radius_m < 0:
            raise ValueError("AutonavGoalSelectionConfig: waypoint_approach_radius_m must be >= 0")
