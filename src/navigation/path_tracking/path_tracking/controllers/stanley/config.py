from dataclasses import dataclass


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

    target_speed_mps: float
    cross_track_gain: float
    front_offset_m: float
    max_steer_rad: float
    max_angular_speed_radps: float
    goal_tolerance_m: float
    max_lateral_accel_mps2: float
    curvature_lookahead_m: float

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
