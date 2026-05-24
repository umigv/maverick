import pathlib
from dataclasses import dataclass


@dataclass(frozen=True)
class OccupancyGridSimulatorConfig:
    """Configuration for the occupancy grid simulator.

    Attributes:
        map_file_path: Path to the JSON map file containing obstacle and lane line cell coordinates.
        width_m: Width of the published occupancy grid in meters (x-axis, forward from robot).
        height_m: Height of the published occupancy grid in meters (y-axis, lateral).
        offset_x_m: X offset (meters) of the grid origin relative to the robot. Negative values place the
            robot inside the grid; 0.0 places the robot at the left edge.
        offset_y_m: Y offset (meters) of the grid origin relative to the robot. Negative values place the
            robot above the bottom edge; -half_height centers the robot vertically.
        map_frame_id: TF frame ID used for the static ground-truth map.
        base_frame_id: TF frame ID used for the robot-centered occupancy grid.
        ground_truth_base_frame_id: TF frame ID of the ground-truth odometry source.
        publish_period_s: Period (seconds) at which the occupancy grid is published.
        robot_blind_spot_height_m: Forward extent (meters) of the triangular blind spot directly in front
            of the robot. Must be positive.
    """

    map_file_path: pathlib.Path
    width_m: float = 5.0
    height_m: float = 5.0
    offset_x_m: float = 0.0
    offset_y_m: float = -2.5
    map_frame_id: str = "map"
    base_frame_id: str = "base_link"
    ground_truth_base_frame_id: str = "base_link_ground_truth"
    publish_period_s: float = 0.03
    robot_blind_spot_height_m: float = 1.25

    def __post_init__(self) -> None:
        if self.robot_blind_spot_height_m <= 0:
            raise ValueError("OccupancyGridSimulatorConfig: robot_blind_spot_height_m must be > 0")
