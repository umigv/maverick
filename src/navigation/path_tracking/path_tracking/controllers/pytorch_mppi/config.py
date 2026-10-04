from dataclasses import dataclass


@dataclass(frozen=True)
class MPPIConfig:
    """Configuration for the MPPI path tracking controller.

    Attributes:
        num_samples: Number of random control sequences sampled per control step.
        horizon_steps: Number of timesteps each sampled sequence is simulated forward.
        dt_s: Duration of one simulated timestep (s). Horizon length is horizon_steps * dt_s.
        temperature: Controls how sharply low-cost samples are favored when blending. Lower values
            trust the best sample more; higher values average more evenly. Passed to the library as lambda_.
        max_linear_speed_mps: Maximum sampled linear velocity (m/s). Also sets the lookahead distance.
        max_angular_speed_radps: Maximum sampled angular velocity (rad/s).
        linear_noise_std_mps: Standard deviation of the random noise added to linear velocity (m/s).
        angular_noise_std_radps: Standard deviation of the random noise added to angular velocity (rad/s).
        lookahead_time_s: Lookahead distance is max_linear_speed_mps * lookahead_time_s (s).
            Ideally matches horizon_steps * dt_s so a full-speed rollout can just reach the lookahead point.
        num_lookahead_poses: Number of evenly spaced poses sampled on the path between the robot
            and the lookahead point. More poses track the path shape more closely but cost more per step.
        goal_tolerance_m: Distance from the final path point at which the robot is considered to have arrived (m).
        following_weight: Cost weight on squared distance between a rollout's endpoint and the lookahead point.
        alignment_weight: Cost weight on squared distance between a rollout and its nearest lookahead pose.
        heading_weight: Cost weight on squared heading difference between a rollout and its nearest lookahead pose.
        smoothing_weight: Cost weight on squared changes between consecutive control commands.
    """

    num_samples: int
    horizon_steps: int
    dt_s: float
    temperature: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    linear_noise_std_mps: float
    angular_noise_std_radps: float
    lookahead_time_s: float
    num_lookahead_poses: int
    goal_tolerance_m: float
    following_weight: float
    alignment_weight: float
    heading_weight: float
    smoothing_weight: float

    def __post_init__(self) -> None:
        if self.num_samples <= 0:
            raise ValueError("MPPIConfig: num_samples must be > 0")
        if self.horizon_steps <= 0:
            raise ValueError("MPPIConfig: horizon_steps must be > 0")
        if self.dt_s <= 0:
            raise ValueError("MPPIConfig: dt_s must be > 0")
        if self.temperature <= 0:
            raise ValueError("MPPIConfig: temperature must be > 0")
        if self.max_linear_speed_mps <= 0:
            raise ValueError("MPPIConfig: max_linear_speed_mps must be > 0")
        if self.max_angular_speed_radps <= 0:
            raise ValueError("MPPIConfig: max_angular_speed_radps must be > 0")
        if self.linear_noise_std_mps <= 0:
            raise ValueError("MPPIConfig: linear_noise_std_mps must be > 0")
        if self.angular_noise_std_radps <= 0:
            raise ValueError("MPPIConfig: angular_noise_std_radps must be > 0")
        if self.lookahead_time_s <= 0:
            raise ValueError("MPPIConfig: lookahead_time_s must be > 0")
        if self.num_lookahead_poses <= 0:
            raise ValueError("MPPIConfig: num_lookahead_poses must be > 0")
        if self.goal_tolerance_m <= 0:
            raise ValueError("MPPIConfig: goal_tolerance_m must be > 0")
        if min(self.following_weight, self.alignment_weight, self.heading_weight, self.smoothing_weight) < 0:
            raise ValueError("MPPIConfig: cost weights must be >= 0")
