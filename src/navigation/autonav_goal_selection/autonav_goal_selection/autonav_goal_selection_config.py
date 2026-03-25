import pathlib
from dataclasses import dataclass


@dataclass(frozen=True)
class GoalSelectionParams:
    """Parameters controlling weighting for goal selection in the occupancy grid."""


@dataclass(frozen=True)
class AutonavGoalSelectionConfig:
    """Configuration for autonomous navigation goal selection.

    Attributes:
        goal_selection_params: Parameters for the goal selection algorithm.
        waypoints_file_path: Path to a JSON file containing the list of GPS waypoints to navigate.
        goal_publish_period_s: How often (seconds) to publish a new local goal.
        waypoint_reached_threshold: Distance (m) within which a waypoint is considered reached.
        map_frame_id: TF frame ID for the map coordinate frame.
        world_frame_id: TF frame ID for the world coordinate frame.
    """

    goal_selection_params: GoalSelectionParams
    waypoints_file_path: pathlib.Path
    goal_publish_period_s: float = 5.0
    waypoint_reached_threshold: float = 1.0
    map_frame_id: str = "map"
    world_frame_id: str = "odom"

    def __post_init__(self) -> None:
        if self.goal_publish_period_s <= 0:
            raise ValueError("AutonavGoalSelectionConfig: goal_publish_period_s must be > 0")
        if self.waypoint_reached_threshold <= 0:
            raise ValueError("AutonavGoalSelectionConfig: waypoint_reached_threshold must be > 0")
