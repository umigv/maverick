from dataclasses import dataclass

from .controllers.controller import Algorithm
from .controllers.differential_drive.config import DifferentialDriveConfig
from .controllers.pure_pursuit.config import PurePursuitConfig
from .controllers.stanley.config import StanleyConfig


@dataclass(frozen=True)
class PathTrackingConfig:
    """Configuration for path tracking.

    Attributes:
        pure_pursuit: Configuration for the pure pursuit controller.
        stanley: Configuration for the Stanley controller.
        differential_drive: Configuration for the differential-drive unicycle controller.
        algorithm: Which controller to run. One of "pure_pursuit", "stanley", "differential_drive".
        control_period_s: Period of the control loop timer (s).
        base_frame_id: Frame ID of the robot base, used as the child frame in odometry validation.
        odom_frame_id: Frame ID of the odometry frame, used to validate incoming odom and path messages.
        ramp_max_speed_m_s: Maximum forward speed (m/s) when in ramp state.
    """

    pure_pursuit: PurePursuitConfig
    stanley: StanleyConfig
    differential_drive: DifferentialDriveConfig
    algorithm: Algorithm = "differential_drive"
    control_period_s: float = 0.01
    base_frame_id: str = "base_link"
    odom_frame_id: str = "odom"
    ramp_max_speed_m_s: float = 1.0

    def __post_init__(self) -> None:
        if self.control_period_s <= 0:
            raise ValueError("PathTrackingConfig: control_period_s must be > 0")
        if self.ramp_max_speed_m_s <= 0:
            raise ValueError("PathTrackingConfig: ramp_max_speed_m_s must be > 0")
