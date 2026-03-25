from dataclasses import dataclass


@dataclass(frozen=True)
class EncVelConfig:
    """Encoder velocity sensor configs.

    Measurement model::
        vx_meas = true_vx * (1 + vx_scale) + N(0, vx_noise_std_mps)
        wz_meas = true_wz * (1 + wz_scale) + N(0, wz_noise_std_radps)

    vx_scale / wz_scale are Ornstein-Uhlenbeck scale-factor biases. The bias is multiplicative, so a stationary robot
    always reads near zero. Odom-frame position drift only accumulates while the robot is moving.

    Attributes:
        vx_noise_std_mps: Per-sample Gaussian noise on linear.x.
        wz_noise_std_radps: Per-sample Gaussian noise on angular.z.
        vx_drift_std: OU scale-factor steady-state std on linear.x.
        wz_drift_std: OU scale-factor steady-state std on angular.z).
        drift_time_constant_s: OU mean-reversion time constant for both scale factors.
    """

    vx_noise_std_mps: float = 0.0
    wz_noise_std_radps: float = 0.0
    vx_drift_std: float = 0.0
    wz_drift_std: float = 0.0
    drift_time_constant_s: float = 30.0

    def __post_init__(self) -> None:
        if self.drift_time_constant_s <= 0:
            raise ValueError("EncVelConfig: drift_time_constant_s must be > 0")


@dataclass(frozen=True)
class Vn300Config:
    """VN-300 INS configs.

    Models the VN-300 as a single sensor: one error state per quantity drives every topic that reports it, matching
    how the real sensor behaves.
    - position drift: shared by gps and odom.pose.position
    - body velocity drift: shared by ins_vel, odom.twist, and imu.angular_velocity
    - yaw drift: shared by imu.orientation and odom.pose.orientation

    Measurement model (per quantity):
        out = true + drift + N(0, noise_std)
    where drift is an Ornstein-Uhlenbeck process with the given steady-state std and time constant.

    Attributes:
        imu_frame_id: TF frame ID for the IMU.
        ins_frame_id: TF frame ID for the INS reference point.

        position_noise_std_m: Per-sample Gaussian noise on horizontal position (m).
        position_drift_std_m: OU position-drift steady-state std (m). Set to 0 to disable drift.
        position_drift_time_constant_s: OU mean-reversion time constant for position drift (s).

        vx_noise_std_mps: Per-sample Gaussian noise on body-frame linear velocity x (m/s).
        wz_noise_std_radps: Per-sample Gaussian noise on body-frame angular velocity z (rad/s).
        vx_drift_std_mps: OU velocity-drift steady-state std on linear.x (m/s).
        wz_drift_std_radps: OU drift steady-state std on angular.z (rad/s).
        vel_drift_time_constant_s: OU mean-reversion time constant for velocity drift (s).

        yaw_noise_std_rad: Per-sample Gaussian noise on absolute yaw (rad).
        yaw_drift_std_rad: OU yaw-drift steady-state std (rad). Set to 0 to disable drift.
        yaw_drift_time_constant_s: OU mean-reversion time constant for yaw drift (s).
    """

    imu_frame_id: str = "imu_link"
    ins_frame_id: str = "base_link"

    position_noise_std_m: float = 0.0
    position_drift_std_m: float = 0.0
    position_drift_time_constant_s: float = 120.0

    vx_noise_std_mps: float = 0.0
    wz_noise_std_radps: float = 0.0
    vx_drift_std_mps: float = 0.0
    wz_drift_std_radps: float = 0.0
    vel_drift_time_constant_s: float = 120.0

    yaw_noise_std_rad: float = 0.0
    yaw_drift_std_rad: float = 0.0
    yaw_drift_time_constant_s: float = 300.0

    @property
    def position_sigma2(self) -> float:
        return self.position_noise_std_m**2 + self.position_drift_std_m**2

    @property
    def yaw_sigma2(self) -> float:
        return self.yaw_noise_std_rad**2 + self.yaw_drift_std_rad**2

    @property
    def vx_sigma2(self) -> float:
        return self.vx_noise_std_mps**2 + self.vx_drift_std_mps**2

    @property
    def wz_sigma2(self) -> float:
        return self.wz_noise_std_radps**2 + self.wz_drift_std_radps**2

    def __post_init__(self) -> None:
        if self.position_drift_time_constant_s <= 0:
            raise ValueError("Vn300Config: position_drift_time_constant_s must be > 0")
        if self.vel_drift_time_constant_s <= 0:
            raise ValueError("Vn300Config: vel_drift_time_constant_s must be > 0")
        if self.yaw_drift_time_constant_s <= 0:
            raise ValueError("Vn300Config: yaw_drift_time_constant_s must be > 0")


@dataclass(frozen=True)
class SensorSimulatorConfig:
    """Top-level configuration for the sensor simulator node.

    Attributes:
        enc_vel: Encoder velocity sensor noise and drift config.
        vn300: VN-300 INS config. Shared error state drives gps, imu, ins_vel, and odom.

        datum: ENU origin as [latitude (deg), longitude (deg), altitude (m)]. Map (0, 0, 0) maps to this point.
        initial_yaw_rad: Initial heading of the robot in map frame in true north ENU (rad).

        map_frame_id: TF frame ID for the map frame.
        base_frame_id: TF frame ID for the robot base frame.
        ground_truth_base_frame_id: TF frame ID for the ground truth robot pose. Use this frame for simulation nodes
            that need the true robot pose rather than the EKF-estimated pose.

        cmd_vel_timeout_s: Seconds without a cmd_vel message before velocity is zeroed (s).
        update_period_s: Publish period for all sensors.
    """

    enc_vel: EncVelConfig
    vn300: Vn300Config

    datum: list[float]
    initial_yaw_rad: float = 0.0

    map_frame_id: str = "map"
    base_frame_id: str = "base_link"
    ground_truth_base_frame_id: str = "base_link_ground_truth"

    cmd_vel_timeout_s: float = 0.5
    update_period_s: float = 0.01

    def __post_init__(self) -> None:
        if len(self.datum) != 3:
            raise ValueError("SensorSimulatorConfig: datum must be [latitude, longitude, altitude]")
        if self.cmd_vel_timeout_s <= 0:
            raise ValueError("SensorSimulatorConfig: cmd_vel_timeout_s must be > 0")
        if self.update_period_s <= 0:
            raise ValueError("SensorSimulatorConfig: update_period_s must be > 0")
