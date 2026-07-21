from dataclasses import dataclass


@dataclass(frozen=True)
class DifferentialDriveConfig:
    """Configuration for the differential-drive unicycle path tracking controller.

    Attributes:
        target_speed_mps: Maximum forward speed (m/s).
        kp_heading: Proportional gain on heading error (rad/s per rad).
        kp_cross: Proportional gain on cross-track error, further scaled by cos(heading_error) (rad/s per m).
        max_angular_speed_radps: Maximum angular velocity command (rad/s).
        max_lateral_speed_mps: Maximum lateral drift speed (m/s). Caps forward speed via v*sin(theta) <= this value,
            preventing forward travel faster than heading can correct.
        heading_lookahead_m: Distance ahead on the path used to compute the heading reference when near corners (m).
            Points toward a lookahead point rather than the local segment tangent, smoothing heading error buildup.
        goal_tolerance_m: Stop when the robot point is within this distance of the final path point (m).
    """

    target_speed_mps: float
    kp_heading: float
    kp_cross: float
    max_angular_speed_radps: float
    max_lateral_speed_mps: float
    heading_lookahead_m: float
    goal_tolerance_m: float

    def __post_init__(self) -> None:
        if self.target_speed_mps <= 0:
            raise ValueError("DifferentialDriveConfig: target_speed_mps must be > 0")
        if self.kp_heading <= 0:
            raise ValueError("DifferentialDriveConfig: kp_heading must be > 0")
        if self.kp_cross <= 0:
            raise ValueError("DifferentialDriveConfig: kp_cross must be > 0")
        if self.max_angular_speed_radps <= 0:
            raise ValueError("DifferentialDriveConfig: max_angular_speed_radps must be > 0")
        if self.max_lateral_speed_mps <= 0:
            raise ValueError("DifferentialDriveConfig: max_lateral_speed_mps must be > 0")
        if self.heading_lookahead_m < 0:
            raise ValueError("DifferentialDriveConfig: heading_lookahead_m must be >= 0")
        if self.goal_tolerance_m <= 0:
            raise ValueError("DifferentialDriveConfig: goal_tolerance_m must be > 0")
