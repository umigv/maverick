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
        mppi: Configuration for the MPPI controller.
        algorithm: Which controller to run. One of "pure_pursuit", "stanley", "differential_drive", "mppi".
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
    # Limits follow odrive_driver: track_width_m=0.764, accel_limit_mps2=3.0, and vel_limit_mps=2.2 (hard trip, so the
    # wheel limit keeps a margin). Angular accel is the wheel accel limit expressed as a pure rotation: 2 * 3.0 / 0.764.
    mppi: MPPIConfig = MPPIConfig(  # noqa: RUF009
        target_speed_mps=2.0,
        max_linear_speed_mps=2.0,
        max_angular_speed_radps=1.5,
        max_linear_accel_mps2=3.0,
        max_angular_accel_radps2=7.85,
        track_width_m=0.764,
        max_wheel_speed_mps=2.0,
        horizon_steps=20,
        timestep_s=0.1,
        num_samples=500,
        temperature=1.0,
        linear_noise_std_mps=0.2,
        angular_noise_std_radps=0.4,
        path_resolution_m=0.2,
        cross_track_weight=20.0,
        heading_weight=3.0,
        speed_weight=2.0,
        linear_rate_weight=1.0,
        angular_rate_weight=0.5,
        progress_weight=5.0,
        terminal_cross_track_weight=20.0,
        goal_tolerance_m=0.3,
        # In simulation the last command led the measured speed by at most 0.095 m/s (8 trials, about 5300 calls while
        # moving), so a resync never fired in normal driving
        speed_resync_threshold_mps=0.3,
        # Reused, halved noise removes call-to-call command chatter (see the MPPI README for the simulation evidence)
        reuse_noise=True,
        noise_refresh_pairs_per_call=1,
        seed=-1,
    )
    algorithm: Algorithm = "differential_drive"
    control_period_s: float = 0.01
    ramp_max_speed_mps: float = 1.0

    def __post_init__(self) -> None:
        if self.control_period_s <= 0:
            raise ValueError("PathTrackingConfig: control_period_s must be > 0")
        if self.ramp_max_speed_mps <= 0:
            raise ValueError("PathTrackingConfig: ramp_max_speed_mps must be > 0")
