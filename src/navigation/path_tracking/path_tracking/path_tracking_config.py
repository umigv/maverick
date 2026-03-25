from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True)
class PurePursuitConfig:
    """Configuration for the pure pursuit path tracking controller.

    Attributes:
        max_angular_speed_radps: Maximum angular velocity command (rad/s).
            Commands are scaled down to stay within this limit.
        base_lookahead_distance_m: Lookahead distance when stationary (m). Added to the speed-dependent term.
        min_lookahead_distance_m: Minimum clamped lookahead distance (m).
        max_lookahead_distance_m: Maximum clamped lookahead distance (m).
        lookahead_speed_gain: Gain applied to current speed when computing adaptive lookahead distance.
        linear_speed_gain: Gain applied to lookahead distance to produce the linear velocity command.
    """

    max_angular_speed_radps: float = 0.6
    base_lookahead_distance_m: float = 0.8
    min_lookahead_distance_m: float = 0.8
    max_lookahead_distance_m: float = 1.5
    lookahead_speed_gain: float = 0.0
    linear_speed_gain: float = 0.5

    def __post_init__(self) -> None:
        if self.max_angular_speed_radps <= 0:
            raise ValueError("PurePursuitConfig: max_angular_speed_radps must be > 0")
        if self.min_lookahead_distance_m <= 0:
            raise ValueError("PurePursuitConfig: min_lookahead_distance_m must be > 0")
        if self.max_lookahead_distance_m <= 0:
            raise ValueError("PurePursuitConfig: max_lookahead_distance_m must be > 0")
        if self.min_lookahead_distance_m > self.max_lookahead_distance_m:
            raise ValueError("PurePursuitConfig: min_lookahead_distance_m must be <= max_lookahead_distance_m")
        if self.linear_speed_gain <= 0:
            raise ValueError("PurePursuitConfig: linear_speed_gain must be > 0")


@dataclass(frozen=True)
class StanleyConfig:
    """Configuration for the Stanley path tracking controller.

    Attributes:
        target_speed_mps: Fixed reference forward speed command (m/s).
        cross_track_gain: Gain applied to cross-track error in the Stanley steering law.
        front_offset_m: Distance ahead of base_link where the virtual front axle is evaluated (m).
            Also reused as the virtual wheelbase for mapping steering angle to angular velocity.
        max_steer_rad: Saturation limit on the commanded steering angle (rad).
        max_angular_speed_radps: Maximum angular velocity command (rad/s).
        goal_tolerance_m: Stop when the front point is within this distance of the final path point (m).
        max_lateral_accel_mps2: Lateral acceleration ceiling used to cap speed in curved sections (m/s^2).
        curvature_lookahead_m: Arclength ahead of the projection over which heading change is accumulated (m).
    """

    target_speed_mps: float = 0.6
    cross_track_gain: float = 0.8
    front_offset_m: float = 0.5
    max_steer_rad: float = 1.2
    max_angular_speed_radps: float = 1.2
    goal_tolerance_m: float = 0.3
    max_lateral_accel_mps2: float = 1.0
    curvature_lookahead_m: float = 0.5

    def __post_init__(self) -> None:
        if self.target_speed_mps <= 0:
            raise ValueError("StanleyConfig: target_speed_mps must be > 0")
        if self.cross_track_gain <= 0:
            raise ValueError("StanleyConfig: cross_track_gain must be > 0")
        if self.front_offset_m <= 0:
            raise ValueError("StanleyConfig: front_offset_m must be > 0")
        if self.max_steer_rad <= 0:
            raise ValueError("StanleyConfig: max_steer_rad must be > 0")
        if self.max_angular_speed_radps <= 0:
            raise ValueError("StanleyConfig: max_angular_speed_radps must be > 0")
        if self.goal_tolerance_m <= 0:
            raise ValueError("StanleyConfig: goal_tolerance_m must be > 0")
        if self.max_lateral_accel_mps2 <= 0:
            raise ValueError("StanleyConfig: max_lateral_accel_mps2 must be > 0")
        if self.curvature_lookahead_m <= 0:
            raise ValueError("StanleyConfig: curvature_lookahead_m must be > 0")


@dataclass(frozen=True)
class PathTrackingConfig:
    """Configuration for path tracking.

    Attributes:
        pure_pursuit: Configuration for the pure pursuit controller.
        stanley: Configuration for the Stanley controller.
        algorithm: Which controller to run. One of "pure_pursuit", "stanley".
        control_period_s: Period of the control loop timer (s).
        base_frame_id: Frame ID of the robot base, used as the child frame in odometry validation.
        odom_frame_id: Frame ID of the odometry frame, used to validate incoming odom and path messages.
    """

    pure_pursuit: PurePursuitConfig
    stanley: StanleyConfig
    algorithm: Literal["pure_pursuit", "stanley"] = "stanley"
    control_period_s: float = 0.01
    base_frame_id: str = "base_link"
    odom_frame_id: str = "odom"

    def __post_init__(self) -> None:
        if self.control_period_s <= 0:
            raise ValueError("PathTrackingConfig: control_period_s must be > 0")
