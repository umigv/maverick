import math
import pathlib
from dataclasses import dataclass


@dataclass(frozen=True)
class PointcloudSimulatorConfig:
    """Configuration for one simulated 3D sensor (lidar or depth camera).

    Attributes:
        map_file_path: Path to the JSON map file containing obstacle cell coordinates.
        map_frame_id: TF frame ID of the ground-truth odometry source's parent frame.
        base_frame_id: TF frame ID of the robot base. Parent of the static sensor transform.
        ground_truth_base_frame_id: TF frame ID of the ground-truth odometry source.
        sensor_frame_id: TF frame ID the point cloud is stamped in. Must be unique per instance.

        offset_x_m: X position (meters) of the sensor relative to base_frame_id.
        offset_y_m: Y position (meters) of the sensor relative to base_frame_id.
        offset_z_m: Z position (meters) of the sensor relative to base_frame_id (height above the ground).
        roll_rad: Roll (radians) of the sensor relative to base_frame_id.
        pitch_rad: Pitch (radians) of the sensor relative to base_frame_id. Positive pitches the sensor down.
        yaw_rad: Yaw (radians) of the sensor relative to base_frame_id.

        horizontal_fov_rad: Horizontal field of view (radians), centered on the sensor's +x axis. 2*pi for a
            spinning lidar.
        vertical_fov_rad: Vertical field of view (radians), centered on the sensor's +x axis.
        horizontal_resolution_rad: Angular spacing (radians) between adjacent rays horizontally.
        vertical_resolution_rad: Angular spacing (radians) between adjacent rays vertically.
        min_range_m: Returns closer than this distance (meters) are dropped.
        max_range_m: Returns farther than this distance (meters) are dropped.
        range_noise_std_m: Standard deviation (meters) of Gaussian noise added along each ray. 0 disables noise.

        obstacle_height_m: Height (meters) that every obstacle cell is extruded to.
        publish_period_s: Period (seconds) at which the point cloud is published.
    """

    map_file_path: pathlib.Path
    map_frame_id: str
    base_frame_id: str
    ground_truth_base_frame_id: str
    sensor_frame_id: str

    offset_x_m: float
    offset_y_m: float
    offset_z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float

    horizontal_fov_rad: float
    vertical_fov_rad: float
    horizontal_resolution_rad: float
    vertical_resolution_rad: float
    min_range_m: float
    max_range_m: float
    range_noise_std_m: float

    obstacle_height_m: float
    publish_period_s: float

    def __post_init__(self) -> None:
        if not 0 < self.horizontal_fov_rad <= 2 * math.pi:
            raise ValueError("PointcloudSimulatorConfig: horizontal_fov_rad must be in (0, 2*pi]")
        if not 0 < self.vertical_fov_rad <= math.pi:
            raise ValueError("PointcloudSimulatorConfig: vertical_fov_rad must be in (0, pi]")
        if self.horizontal_resolution_rad <= 0:
            raise ValueError("PointcloudSimulatorConfig: horizontal_resolution_rad must be > 0")
        if self.vertical_resolution_rad <= 0:
            raise ValueError("PointcloudSimulatorConfig: vertical_resolution_rad must be > 0")
        if not 0 <= self.min_range_m < self.max_range_m:
            raise ValueError("PointcloudSimulatorConfig: must have 0 <= min_range_m < max_range_m")
        if self.range_noise_std_m < 0:
            raise ValueError("PointcloudSimulatorConfig: range_noise_std_m must be >= 0")
        if self.obstacle_height_m <= 0:
            raise ValueError("PointcloudSimulatorConfig: obstacle_height_m must be > 0")
        if self.publish_period_s <= 0:
            raise ValueError("PointcloudSimulatorConfig: publish_period_s must be > 0")
