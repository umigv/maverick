import pathlib
from dataclasses import dataclass


@dataclass(frozen=True)
class AutonavMissionControlConfig:
    """Configuration for the autonav mission control node.

    Attributes:
        waypoints_file_path: Path to a JSON file containing the ordered list of waypoints.
        waypoint_reached_threshold_m: Distance (m) within which a waypoint is considered reached.
        ramp_approach_radius_m: Distance (m) from a ramp_approach waypoint at which ramp mode activates.
        lane_detection_enable_near_exit_radius_m: Distance (m) from a no_mans_land_exit waypoint within which lane
            detection is re-enabled. Allows the robot to see lane markings as it approaches the zone exit.
        mission_complete_delay_s: Time (s) after the final waypoint is reached before mission_complete is set. Normal
            goal selection continues during this window.
        map_frame_id: TF frame ID for the map coordinate frame. Global odometry must publish in this frame.
    """

    waypoints_file_path: pathlib.Path
    waypoint_reached_threshold_m: float = 0.5
    ramp_approach_radius_m: float = 12.0
    lane_detection_enable_near_exit_radius_m: float = 3.0
    mission_complete_delay_s: float = 10.0
    map_frame_id: str = "map"

    def __post_init__(self) -> None:
        if self.waypoint_reached_threshold_m <= 0:
            raise ValueError("AutonavMissionControlConfig: waypoint_reached_threshold_m must be > 0")
        if self.ramp_approach_radius_m <= 0:
            raise ValueError("AutonavMissionControlConfig: ramp_approach_radius_m must be > 0")
        if self.lane_detection_enable_near_exit_radius_m < 0:
            raise ValueError("AutonavMissionControlConfig: lane_detection_enable_near_exit_radius_m must be >= 0")
        if self.mission_complete_delay_s < 0:
            raise ValueError("AutonavMissionControlConfig: mission_complete_delay_s must be >= 0")
