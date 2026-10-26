import itertools
import pathlib
from dataclasses import dataclass, field


@dataclass(frozen=True)
class OccupancyGridSimulatorConfig:
    """Configuration for the occupancy grid simulator.

    Attributes:
        map_file_path: Path to the JSON map file containing obstacle and lane line cell coordinates.
        map_frame_id: TF frame ID used for the static ground-truth map.
        base_frame_id: TF frame ID used for the robot-centered occupancy grid.
        ground_truth_base_frame_id: TF frame ID of the ground-truth odometry source.
        width_m: Width of the published occupancy grid in meters (x-axis, forward from robot).
        height_m: Height of the published occupancy grid in meters (y-axis, lateral).
        offset_x_m: X offset (meters) of the grid origin relative to the robot. Negative values place the
            robot inside the grid; 0.0 places the robot at the left edge.
        offset_y_m: Y offset (meters) of the grid origin relative to the robot. Negative values place the
            robot above the bottom edge; -half_height centers the robot vertically.
        publish_period_s: Period (seconds) at which the occupancy grid is published.
        robot_blind_spot_boundary_lateral_m: Lateral (y, +left) positions in meters of the vertices of the
            blind spot's far boundary, in the robot frame. Must be strictly increasing and have at least two entries.
        robot_blind_spot_boundary_forward_m: Forward (x) positions in meters of the same vertices. A cell is in the
            blind spot if it is closer to the robot (smaller x) than the boundary interpolated at its lateral
            position; outside the lateral range the nearest vertex's value is used. The defaults are fitted to
            the merged two-camera grid recorded on the real robot.
    """

    map_file_path: pathlib.Path
    map_frame_id: str
    base_frame_id: str
    ground_truth_base_frame_id: str
    width_m: float = 5.0
    height_m: float = 5.0
    offset_x_m: float = 0.0
    offset_y_m: float = -2.5
    publish_period_s: float = 0.03
    # list of x and list of y rather than list of Point2d because ROS parameters
    # cannot hold a list of points.
    robot_blind_spot_boundary_lateral_m: list[float] = field(default_factory=lambda: [-2.5, -1.22, -0.18, 1.45, 2.5])
    robot_blind_spot_boundary_forward_m: list[float] = field(default_factory=lambda: [1.12, 0.49, 0.77, 0.44, 1.04])

    def __post_init__(self) -> None:
        lateral_m = self.robot_blind_spot_boundary_lateral_m
        forward_m = self.robot_blind_spot_boundary_forward_m
        if len(lateral_m) != len(forward_m):
            raise ValueError(
                "OccupancyGridSimulatorConfig: robot_blind_spot_boundary_lateral_m and "
                "robot_blind_spot_boundary_forward_m must have the same length"
            )
        if len(lateral_m) < 2:
            raise ValueError("OccupancyGridSimulatorConfig: robot blind spot boundary must have at least 2 vertices")
        if any(left <= right for right, left in itertools.pairwise(lateral_m)):
            raise ValueError(
                "OccupancyGridSimulatorConfig: robot_blind_spot_boundary_lateral_m must be strictly increasing"
            )
        if any(forward < 0 for forward in forward_m):
            raise ValueError("OccupancyGridSimulatorConfig: robot_blind_spot_boundary_forward_m must be >= 0")
