from dataclasses import dataclass


@dataclass(frozen=True)
class UltrasonicSimulatorConfig:
    """Configuration for the ultrasonic sensor simulator node.

    Subscribes to the ground truth odometry and obstacle occupancy grid, raycasts from each
    sensor's position in the map frame, then merges all readings into one by taking the minimum
    and publishes a single sensor_msgs/Range on ``topic``.

    Measurement model::
        range_meas = raycast_distance + N(0, noise_std_m), clamped to [min_range_m, max_range_m]
        merged     = min(range_meas for each sensor)

    Attributes:
        frame_ids: TF frame IDs of the sensor transducer faces (e.g. the ``${name}_link`` frames
            from the ultrasonic xacro).  Used for raycasting only — does not affect the topic.

        topic: Topic name on which the merged Range message is published.

        min_range_m: Minimum measurable distance (m).  Readings below this are clamped.
        max_range_m: Maximum measurable distance (m).  Readings above this are reported as
            max_range_m.
        field_of_view_rad: Total beam cone angle (rad).  Written into Range.field_of_view
            metadata only — raycasting uses the beam centre-line.
        noise_std_m: Gaussian range noise standard deviation (m).

        map_frame_id: TF frame ID for the world/map frame.
        base_frame_id: TF frame ID for the robot base link.  Used to look up the static
            sensor-to-base transforms.
        ground_truth_base_frame_id: Expected child_frame_id in ground truth odometry messages.

        publish_period_s: Publish interval (s).
    """

    frame_ids: list[str]

    topic: str = "ultrasonic_rear"

    min_range_m: float = 0.02
    max_range_m: float = 4.0
    field_of_view_rad: float = 0.5236  # ~30 degrees
    noise_std_m: float = 0.005

    map_frame_id: str = "map"
    base_frame_id: str = "base_link"
    ground_truth_base_frame_id: str = "base_link_ground_truth"

    publish_period_s: float = 0.05  # 20 Hz

    def __post_init__(self) -> None:
        if not self.frame_ids:
            raise ValueError("UltrasonicSimulatorConfig: frame_ids must not be empty")
        if self.min_range_m >= self.max_range_m:
            raise ValueError("UltrasonicSimulatorConfig: min_range_m must be < max_range_m")
        if self.publish_period_s <= 0:
            raise ValueError("UltrasonicSimulatorConfig: publish_period_s must be > 0")
