from dataclasses import dataclass


@dataclass(frozen=True)
class PathPlanningConfig:
    """Configuration for the path planning node.

    Attributes:
        frame_id: TF frame ID for the occupancy grid, odometry, goal, and published path.
        max_unknown_forward_m: How far forward of the robot (in the robot's frame) unknown cells are treated as
            traversable by A*. Allows planning directly from the robot's position through the unknown region that
            sensors cannot observe directly beneath the robot.
        max_unknown_sideways_m: How far sideways of the robot unknown cells are treated as traversable by A*.
            Kept smaller than max_unknown_forward_m to prevent the planner from escaping sideways into unbounded
            out-of-bounds unknown space.
        line_of_sight_step_m: Step size in meters used when sampling segments during string-pulling line-of-sight
            checks. Smaller values give more accurate collision checking at the cost of more computation.
    """

    frame_id: str = "odom"
    max_unknown_forward_m: float = 2.0
    max_unknown_sideways_m: float = 1.0
    line_of_sight_step_m: float = 0.05

    def __post_init__(self) -> None:
        if self.max_unknown_forward_m <= 0:
            raise ValueError("PathPlanningConfig: max_unknown_forward_m must be > 0.")
        if self.max_unknown_sideways_m <= 0:
            raise ValueError("PathPlanningConfig: max_unknown_sideways_m must be > 0.")
        if self.line_of_sight_step_m <= 0:
            raise ValueError("PathPlanningConfig: line_of_sight_step_m must be > 0.")
