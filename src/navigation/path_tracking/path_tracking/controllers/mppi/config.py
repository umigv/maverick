from dataclasses import dataclass


@dataclass(frozen=True)
class MPPILimitsConfig:
    """The Physical limit of the robot.

    Attributes:
        max_linear_speed_mps: Max forward speed command (m/s). Reverse is never commanded
        max_angular_speed_radps: Max angular velocity command (rad/s)
        max_linear_accel_mps2: Max change in linear velocity per second (m/s^2).
        max_angular_accel_radps2: Max change in angular velocity per second (rad/s^2).
        track_width_m: Distance between the left and right wheels (m), used for the wheel-speed limit.
        max_wheel_speed_mps: Max speed of either wheel, |v -/+ omega * track_width_m / 2| (m/s).
    """

    max_linear_speed_mps: float
    max_angular_speed_radps: float
    max_linear_accel_mps2: float
    max_angular_accel_radps2: float
    track_width_m: float
    max_wheel_speed_mps: float

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if value <= 0:
                raise ValueError(f"MPPIConfig.limits: {name} must be > 0")


@dataclass(frozen=True)
class MPPISamplingConfig:
    """How many candidate plans to try each update and how far ahead they look.

    Attributes:
        horizon_steps: Number of timesteps in each rollout.
        timestep_s: Duration of one rollout timestep (s). The prediction horizon is horizon_steps * timestep_s.
        num_samples: Number of sampled control sequences per update. Even, since samples come in mirrored pairs.
        temperature: MPPI temperature lambda. Higher value make the update more uniform.
    """

    horizon_steps: int
    timestep_s: float  #
    num_samples: int
    temperature: float

    def __post_init__(self) -> None:
        for name in ("horizon_steps", "timestep_s", "temperature"):
            if getattr(self, name) <= 0:
                raise ValueError(f"MPPIConfig.sampling: {name} must be > 0")
        # Noise is drawn in mirrored pairs and sample 0 stays the unperturbed nominal
        if self.num_samples < 4 or self.num_samples % 2:
            raise ValueError("MPPIConfig.sampling: num_samples must be even and >= 4")


@dataclass(frozen=True)
class MPPINoiseConfig:
    """The random wiggle added to the current plan to get the candidate plans.

    Attributes:
        linear_noise_std_mps: Standard deviation of the linear velocity exploration noise (m/s).
        angular_noise_std_radps: Standard deviation of the angular velocity exploration noise (rad/s).
        seed: Random seed for the exploration noise. -1 = random, 0 = fixed, >0 = fixed with that seed.
    """

    linear_noise_std_mps: float
    angular_noise_std_radps: float
    seed: int

    def __post_init__(self) -> None:
        for name in ("linear_noise_std_mps", "angular_noise_std_radps"):
            if getattr(self, name) <= 0:
                raise ValueError(f"MPPIConfig.noise: {name} must be > 0")
        if self.seed < -1:
            raise ValueError("MPPIConfig.noise: seed must be >= -1")


@dataclass(frozen=True)
class MPPICostConfig:
    """How rollouts are scored. Running weights are per second of the horizon; terminal ones apply once at its end.

    Attributes:
        path_resolution_m: Spacing the path is resampled to for cost evaluation (m). Smaller is more accurate but slower.
        cross_track_weight: Running cost weight on squared distance to the path (per m^2 s).
        heading_weight: Running cost weight on squared heading error to the path tangent (per rad^2 s).
        speed_weight: Running cost weight on squared error to the reference speed (per (m/s)^2 s).
        linear_rate_weight: Running cost weight on squared per-step change in linear velocity (per (m/s)^2 s).
        angular_rate_weight: Running cost weight on squared per-step change in angular velocity (per (rad/s)^2 s).
        progress_weight: Terminal reward per meter of arc length progressed along the path (per m).
        terminal_cross_track_weight: Terminal cost weight on squared distance to the path at the end of the horizon
            (per m^2).
    """

    path_resolution_m: float
    cross_track_weight: float
    heading_weight: float
    speed_weight: float
    linear_rate_weight: float
    angular_rate_weight: float
    progress_weight: float
    terminal_cross_track_weight: float

    def __post_init__(self) -> None:
        if self.path_resolution_m <= 0:
            raise ValueError("MPPIConfig.cost: path_resolution_m must be > 0")
        for name, value in vars(self).items():
            if value < 0:
                raise ValueError(f"MPPIConfig.cost: {name} must be >= 0")


@dataclass(frozen=True)
class MPPIConfig:
    """Configuration for the MPPI path tracking controller.

    Attributes:
        target_speed_mps: Desired forward speed along the path (m/s). Tapered near the goal so the robot can stop without jerking.
        goal_tolerance_m: Stop when the robot point is within this distance of the final path point (m).
        speed_resync_threshold_mps: If the measured speed is lower than the last published command by more than this,
            the stored command is reset to the measured speed before optimizing (m/s). Covers commands that were changed
            or never applied downstream (the node's ramp speed cap, twist_mux handing control to recovery or teleop)
        limits: Speed, acceleration and wheel-speed limits
        sampling: Number of samples, horizon and temperature
        noise: Exploration noise
        cost: Cost weights
    """

    target_speed_mps: float
    goal_tolerance_m: float
    speed_resync_threshold_mps: float
    limits: MPPILimitsConfig
    sampling: MPPISamplingConfig
    noise: MPPINoiseConfig
    cost: MPPICostConfig

    def __post_init__(self) -> None:
        for name in ("target_speed_mps", "goal_tolerance_m", "speed_resync_threshold_mps"):
            if getattr(self, name) <= 0:
                raise ValueError(f"MPPIConfig: {name} must be > 0")
        if self.target_speed_mps > min(self.limits.max_linear_speed_mps, self.limits.max_wheel_speed_mps):
            raise ValueError("MPPIConfig: target_speed_mps must be <= max_linear_speed_mps and max_wheel_speed_mps")
