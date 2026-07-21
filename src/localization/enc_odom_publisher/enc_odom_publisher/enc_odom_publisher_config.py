from dataclasses import dataclass


@dataclass(frozen=True)
class EncOdomPublisherConfig:
    """Config for EncOdomPublisher.

    Attributes:
        odom_frame_id: Parent frame for odometry and TF.
        base_frame_id: Child frame for odometry and TF.
        pose_x_variance_m2: Pose covariance for x (m^2).
        pose_y_variance_m2: Pose covariance for y (m^2).
        pose_yaw_variance_rad2: Pose covariance for yaw (rad^2).
        max_dt_s: Maximum allowed dt (s) between encoder messages; updates exceeding this are dropped.
        publish_period_s: Period (s) at which odom and TF are published.
    """

    odom_frame_id: str
    base_frame_id: str
    pose_x_variance_m2: float = 0.01
    pose_y_variance_m2: float = 0.01
    pose_yaw_variance_rad2: float = 0.01
    max_dt_s: float = 1.0
    publish_period_s: float = 0.01

    def __post_init__(self) -> None:
        if self.pose_x_variance_m2 <= 0:
            raise ValueError("EncOdomPublisherConfig: pose_x_variance_m2 must be > 0")
        if self.pose_y_variance_m2 <= 0:
            raise ValueError("EncOdomPublisherConfig: pose_y_variance_m2 must be > 0")
        if self.pose_yaw_variance_rad2 <= 0:
            raise ValueError("EncOdomPublisherConfig: pose_yaw_variance_rad2 must be > 0")
        if self.max_dt_s <= 0:
            raise ValueError("EncOdomPublisherConfig: max_dt_s must be > 0")
        if self.publish_period_s <= 0:
            raise ValueError("EncOdomPublisherConfig: publish_period_s must be > 0")
