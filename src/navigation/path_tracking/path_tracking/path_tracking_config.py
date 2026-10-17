from dataclasses import dataclass

from .controllers.controller import Algorithm
from .controllers.differential_drive.config import DifferentialDriveConfig
from .controllers.mppi.config import MPPIConfig
from .controllers.pure_pursuit.config import PurePursuitConfig
from .controllers.stanley.config import StanleyConfig


@dataclass(frozen=True)
class PathTrackingConfig:
    """Configuration for path tracking.

    Attributes:
        base_frame_id: Frame ID of the robot base, used as the child frame in odometry validation.
        odom_frame_id: Frame ID of the odometry frame, used to validate incoming odom and path messages.
        pure_pursuit: Configuration for the pure pursuit controller.
        stanley: Configuration for the Stanley controller.
        differential_drive: Configuration for the differential-drive unicycle controller.
        mppi: Configuration for the Model Predictive Path Integral controller.
        algorithm: Which controller to run. One of "pure_pursuit", "stanley", "differential_drive".
        control_period_s: Period of the control loop timer (s).
        ramp_max_speed_mps: Maximum forward speed (m/s) while the mission state has in_ramp_approach set.
    """

    base_frame_id: str
    odom_frame_id: str
    # The controller configs are frozen dataclasses, so sharing one default instance is safe. RUF009 flags them only
    # because it cannot verify immutability across modules.
    pure_pursuit: PurePursuitConfig = PurePursuitConfig(  # noqa: RUF009
        max_angular_speed_radps=0.6,
        base_lookahead_distance_m=0.8,
        min_lookahead_distance_m=0.8,
        max_lookahead_distance_m=1.5,
        lookahead_speed_gain=0.0,
        linear_speed_gain=0.5,
    )
    stanley: StanleyConfig = StanleyConfig(  # noqa: RUF009
        target_speed_mps=1.35,
        cross_track_gain=0.6,
        front_offset_m=0.85,
        max_steer_rad=1.2,
        max_angular_speed_radps=1.0,
        goal_tolerance_m=0.3,
        max_lateral_accel_mps2=1.0,
        curvature_lookahead_m=1.5,
    )
    differential_drive: DifferentialDriveConfig = DifferentialDriveConfig(  # noqa: RUF009
        target_speed_mps=1.35,
        kp_heading=0.8,
        kp_cross=1.0,
        max_angular_speed_radps=1.5,
        max_lateral_speed_mps=0.4,
        heading_lookahead_m=1.0,
        goal_tolerance_m=0.3,
    )
    mppi: MPPIConfig = MPPIConfig(  # noqa: RUF009
        dt=0.1,
        steps=20,
        num_samples=100,
        temperature=1.0,
        linear_noise_std=0.1,
        angular_noise_std=0.1,
        target_speed_mps=1.0,
        min_speed_mps=0.4,
        max_speed_mps=2.0,
        max_angular_speed_radps=0.6,
        goal_tolerance_m=0.2,
        slowdown_distance_m=1.0,
        goal_speed_tolerance=0.05,
        path_error_scale_m=0.2,
        heading_error_scale_rad=0.5,
        speed_error_scale_mps=0.4,
        linear_command_change_scale_mps=0.1,
        angular_command_change_scale_radps=0.2,
        terminal_goal_error_scale_m=0.5,
        path_distance_weight=5.0,
        speed_weight=1.0,
        linear_smoothness_weight=0.5,
        angular_smoothness_weight=0.5,
    )
    algorithm: Algorithm = "mppi"
    control_period_s: float = 0.1
    ramp_max_speed_mps: float = 1.0

    def __post_init__(self) -> None:
        if self.control_period_s <= 0:
            raise ValueError("PathTrackingConfig: control_period_s must be > 0")
        if self.ramp_max_speed_mps <= 0:
            raise ValueError("PathTrackingConfig: ramp_max_speed_mps must be > 0")
