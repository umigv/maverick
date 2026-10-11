from dataclasses import dataclass


@dataclass(frozen=True)
class MPPIConfig:
    """Configuration for the MPPI (Model Predictive Path Integral) path tracking controller.

    Attributes:
        target_speed_mps: Desired forward speed along the path (m/s). Tapered near the goal so the robot can stop.
        max_linear_speed_mps: Maximum forward speed command (m/s). Reverse is never commanded.
        max_angular_speed_radps: Maximum angular velocity command (rad/s).
        max_linear_accel_mps2: Maximum change in linear velocity per second (m/s^2), enforced in rollouts and on the
            output command.
        max_angular_accel_radps2: Maximum change in angular velocity per second (rad/s^2), enforced in rollouts and on
            the output command.
        track_width_m: Distance between the left and right wheels (m), used for the wheel-speed limit.
        max_wheel_speed_mps: Maximum speed of either wheel, |v -/+ omega * track_width_m / 2| (m/s).
        horizon_steps: Number of timesteps in each rollout.
        timestep_s: Duration of one rollout timestep (s). The prediction horizon is horizon_steps * timestep_s.
        num_samples: Number of sampled control sequences per update.
        temperature: MPPI temperature lambda. Lower values weight the lowest-cost samples more aggressively.
        linear_noise_std_mps: Standard deviation of the linear velocity exploration noise (m/s).
        angular_noise_std_radps: Standard deviation of the angular velocity exploration noise (rad/s).
        path_resolution_m: Spacing the path is resampled to for cost evaluation (m). Smaller is more accurate but slower.
        cross_track_weight: Running cost weight on squared distance to the path (per m^2 s).
        heading_weight: Running cost weight on squared heading error to the path tangent (per rad^2 s).
        speed_weight: Running cost weight on squared error to the reference speed (per (m/s)^2 s).
        linear_rate_weight: Running cost weight on squared per-step change in linear velocity (per (m/s)^2 s).
        angular_rate_weight: Running cost weight on squared per-step change in angular velocity (per (rad/s)^2 s).
        progress_weight: Terminal reward per meter of arc length progressed along the path (per m).
        terminal_cross_track_weight: Terminal cost weight on squared distance to the path at the end of the horizon
            (per m^2).
        goal_tolerance_m: Stop when the robot point is within this distance of the final path point (m).
        speed_resync_threshold_mps: If the measured speed is lower than the last published command by more than this,
            the stored command is reset to the measured speed before optimizing (m/s). Covers commands that were changed
            or never applied downstream (the node's ramp speed cap, twist_mux handing control to recovery or teleop).
            Must exceed the drive's normal speed tracking lag, or MPPI cannot accelerate past the measured speed.
        reuse_noise: Keep the exploration noise between updates instead of drawing it fresh every call, replacing one
            mirrored pair of samples per call. Fresh noise makes the weighted update jump from call to call, which the
            rate limits turn into command chatter. Pair with lower noise standard deviations.
        noise_refresh_pairs_per_call: With reuse_noise, how many sample pairs are redrawn per call. More renews the
            sample set faster (less run-to-run variation) at the cost of more call-to-call change.
        seed: Random seed for the exploration noise. -1 seeds from OS entropy.
    """

    target_speed_mps: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    max_linear_accel_mps2: float
    max_angular_accel_radps2: float
    track_width_m: float
    max_wheel_speed_mps: float
    horizon_steps: int
    timestep_s: float
    num_samples: int
    temperature: float
    linear_noise_std_mps: float
    angular_noise_std_radps: float
    path_resolution_m: float
    cross_track_weight: float
    heading_weight: float
    speed_weight: float
    linear_rate_weight: float
    angular_rate_weight: float
    progress_weight: float
    terminal_cross_track_weight: float
    goal_tolerance_m: float
    speed_resync_threshold_mps: float
    reuse_noise: bool
    noise_refresh_pairs_per_call: int
    seed: int

    def __post_init__(self) -> None:
        positive = (
            "target_speed_mps",
            "max_linear_speed_mps",
            "max_angular_speed_radps",
            "max_linear_accel_mps2",
            "max_angular_accel_radps2",
            "track_width_m",
            "max_wheel_speed_mps",
            "horizon_steps",
            "timestep_s",
            "temperature",
            "linear_noise_std_mps",
            "angular_noise_std_radps",
            "path_resolution_m",
            "goal_tolerance_m",
            "speed_resync_threshold_mps",
        )
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError(f"MPPIConfig: {name} must be > 0")
        non_negative = (
            "cross_track_weight",
            "heading_weight",
            "speed_weight",
            "linear_rate_weight",
            "angular_rate_weight",
            "progress_weight",
            "terminal_cross_track_weight",
        )
        for name in non_negative:
            if getattr(self, name) < 0:
                raise ValueError(f"MPPIConfig: {name} must be >= 0")
        if self.num_samples < 2:
            raise ValueError("MPPIConfig: num_samples must be >= 2")
        if self.reuse_noise:
            # Reused noise is drawn in mirrored pairs and sample 0 stays the unperturbed nominal
            if self.num_samples < 4 or self.num_samples % 2:
                raise ValueError("MPPIConfig: num_samples must be even and >= 4 when reuse_noise is set")
            if not 1 <= self.noise_refresh_pairs_per_call < self.num_samples // 2:
                raise ValueError("MPPIConfig: noise_refresh_pairs_per_call must be >= 1 and < num_samples / 2")
        if self.target_speed_mps > min(self.max_linear_speed_mps, self.max_wheel_speed_mps):
            raise ValueError("MPPIConfig: target_speed_mps must be <= max_linear_speed_mps and max_wheel_speed_mps")
        if self.seed < -1:
            raise ValueError("MPPIConfig: seed must be >= -1")
