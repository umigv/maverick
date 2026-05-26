from dataclasses import dataclass

from .controllers.controller import Algorithm
from .controllers.pure_pursuit.config import PurePursuitConfig
from .controllers.stanley.config import StanleyConfig


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
    algorithm: Algorithm = "stanley"
    control_period_s: float = 0.01
    base_frame_id: str = "base_link"
    odom_frame_id: str = "odom"

    def __post_init__(self) -> None:
        if self.control_period_s <= 0:
            raise ValueError("PathTrackingConfig: control_period_s must be > 0")
