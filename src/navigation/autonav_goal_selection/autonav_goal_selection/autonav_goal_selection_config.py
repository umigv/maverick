import math
import pathlib
from dataclasses import dataclass


@dataclass(frozen=True)
class GoalSelectionParams:
    """Parameters controlling the ray-cast goal selection heuristic.

    Attributes:
        num_rays: Number of rays cast across the forward arc.
        arc_half_angle_rad: Half-angle (rad) of the forward arc; rays span [-h, +h] from heading.
        max_ray_length_m: Maximum free length walked along each ray.
        step_size_m: Step size used when walking each ray; should be ~grid resolution.
        alignment_exponent: Sharpness of the multiplicative waypoint-alignment factor
            `((1 + cos(angle_to_waypoint)) / 2) ** alignment_exponent`. 0 disables the bias
            entirely; 1 gives a smooth half-cosine; larger values penalize off-axis rays
            more aggressively (e.g. 2 → factor of 0.25 at 90° off, 0 at 180°).
        min_goal_progress_m: Minimum free length the chosen ray must have for a goal to be
            published. If the highest-scoring ray's free_length is below this, `select_goal`
            returns no goal (caller should treat as "stuck"). Set to 0 to always publish.
        neighbor_smoothing_window: Number of neighbors on each side averaged into each ray's
            score before picking. 0 disables smoothing.
        safety_margin_m: Distance (m) the chosen endpoint is pulled back from where the ray
            terminated.
        max_unknown_forward_m: Mirror of path_planning: how far forward unknown cells are
            considered drivable when walking a ray.
        max_unknown_sideways_m: Mirror of path_planning: how far sideways unknown cells are
            considered drivable when walking a ray.
    """

    num_rays: int = 74
    arc_half_angle_rad: float = math.pi / 2
    max_ray_length_m: float = 100
    step_size_m: float = 0.05
    alignment_exponent: float = 0.0
    min_goal_progress_m: float = 0.2
    neighbor_smoothing_window: int = 2
    safety_margin_m: float = 0.1
    max_unknown_forward_m: float = 2.0
    max_unknown_sideways_m: float = 1.0

    def __post_init__(self) -> None:
        if self.num_rays < 2:
            raise ValueError("GoalSelectionParams: num_rays must be >= 2")
        if not (0 < self.arc_half_angle_rad <= math.pi):
            raise ValueError("GoalSelectionParams: arc_half_angle_rad must be in (0, pi]")
        if self.max_ray_length_m <= 0:
            raise ValueError("GoalSelectionParams: max_ray_length_m must be > 0")
        if self.step_size_m <= 0:
            raise ValueError("GoalSelectionParams: step_size_m must be > 0")
        if self.alignment_exponent < 0:
            raise ValueError("GoalSelectionParams: alignment_exponent must be >= 0")
        if self.min_goal_progress_m < 0:
            raise ValueError("GoalSelectionParams: min_goal_progress_m must be >= 0")
        if self.neighbor_smoothing_window < 0:
            raise ValueError("GoalSelectionParams: neighbor_smoothing_window must be >= 0")
        if self.safety_margin_m < 0:
            raise ValueError("GoalSelectionParams: safety_margin_m must be >= 0")
        if self.max_unknown_forward_m <= 0:
            raise ValueError("GoalSelectionParams: max_unknown_forward_m must be > 0")
        if self.max_unknown_sideways_m <= 0:
            raise ValueError("GoalSelectionParams: max_unknown_sideways_m must be > 0")


@dataclass(frozen=True)
class AutonavGoalSelectionConfig:
    """Configuration for autonomous navigation goal selection.

    Attributes:
        goal_selection_params: Parameters for the ray-cast goal selection algorithm.
        waypoints_file_path: Path to a JSON file containing the list of map-frame waypoints.
            Expected format: {"waypoints": [{"x": <float>, "y": <float>}, ...]}.
        goal_publish_period_s: How often (seconds) to publish a new local goal.
        waypoint_reached_threshold: Distance (m) within which a waypoint is considered reached.
        map_frame_id: TF frame ID for the map coordinate frame.
        world_frame_id: TF frame ID for the world coordinate frame.
        publish_debug: When true, publish a MarkerArray on `goal_selection_debug` showing all
            rays, the chosen ray, the chosen endpoint, and the waypoint direction.
    """

    goal_selection_params: GoalSelectionParams
    waypoints_file_path: pathlib.Path
    goal_publish_period_s: float = 0.5
    waypoint_reached_threshold: float = 1.0
    map_frame_id: str = "map"
    world_frame_id: str = "odom"
    publish_debug: bool = True

    def __post_init__(self) -> None:
        if self.goal_publish_period_s <= 0:
            raise ValueError("AutonavGoalSelectionConfig: goal_publish_period_s must be > 0")
        if self.waypoint_reached_threshold <= 0:
            raise ValueError("AutonavGoalSelectionConfig: waypoint_reached_threshold must be > 0")
