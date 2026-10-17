from dataclasses import dataclass


@dataclass(frozen=True)
class MPPIConfig:
    """Configuration for the MPPI tracking controller.

    Attributes:
        FIXME: Add MPPI configuration fields.
    """

    # Each step size (seconds) for the MPPI model, and the number of steps
    # looked into future. How long into future = dt * steps (seconds)
    dt: float = 0.1
    steps: int = 20

    # number of samples for each step
    num_samples: int = 100

    # high temperature = lower weight on low cost paths
    # lower temperature = higher weight on low cost paths
    temperature: float = 1.0

    linear_noise_std: float = 0.1
    angular_noise_std: float = 0.1

    target_speed_mps: float = 2.0
    min_speed_mps: float = 0.0
    max_speed_mps: float = 3.0
    max_angular_speed_radps: float = 0.6

    # max_linear_accel = TODO: add max accel
    # max_linear_decel = TODO: add max decel
    # max_angular_accel = TODO: add max angular accel

    # how close to goal = reached goal
    goal_tolerance_m: float = 0.2

    # how far from goal to start slowing down
    slowdown_distance_m: float = 1.0

    # how close to the target speed = reached target speed
    goal_speed_tolerance: float = 0.05

    # Error normalization scales (consider just adding this to costs instead?)
    path_error_scale_m: float = 0.2
    heading_error_scale_rad: float = 0.5
    speed_error_scale_mps: float = 0.4
    linear_command_change_scale_mps: float = 0.1
    angular_command_change_scale_radps: float = 0.2
    terminal_goal_error_scale_m: float = 0.5

    path_distance_weight: float = 5.0
    speed_weight: float = 1.0  # consider changing to dist from goal later
    heading_weight: float = 1.0

    # how important the smoothness of movement is
    linear_smoothness_weight: float = 0.5
    angular_smoothness_weight: float = 0.5

    # consider adding later: terminal_goal_weight: float = 2.0

    def __post_init__(self) -> None:
        if self.dt <= 0:
            raise ValueError("MPPIConfig: dt must be > 0")
        if self.steps <= 0:
            raise ValueError("MPPIConfig: steps must be > 0")
        if self.num_samples <= 0:
            raise ValueError("MPPIConfig: num_samples must be > 0")
        if self.temperature <= 0:
            raise ValueError("MPPIConfig: temperature must be > 0")
        if self.linear_noise_std < 0:
            raise ValueError("MPPIConfig: linear_noise_std must be >= 0")
        if self.angular_noise_std < 0:
            raise ValueError("MPPIConfig: angular_noise_std must be >= 0")
        if self.target_speed_mps <= 0:
            raise ValueError("MPPIConfig: target_speed_mps must be > 0")
        if self.min_speed_mps < 0:
            raise ValueError("MPPIConfig: min_speed_mps must be >= 0")
        if self.max_speed_mps <= 0:
            raise ValueError("MPPIConfig: max_speed_mps must be > 0")
        if self.max_angular_speed_radps <= 0:
            raise ValueError("MPPIConfig: max_angular_speed_radps must be > 0")
        if self.goal_tolerance_m <= 0:
            raise ValueError("MPPIConfig: goal_tolerance_m must be > 0")
        if self.slowdown_distance_m < 0:
            raise ValueError("MPPIConfig: slowdown_distance_m must be >= 0")
        if self.goal_speed_tolerance < 0:
            raise ValueError("MPPIConfig: goal_speed_tolerance must be >= 0")
        if self.path_error_scale_m <= 0:
            raise ValueError("MPPIConfig: path_error_scale_m must be > 0")
        if self.heading_error_scale_rad <= 0:
            raise ValueError("MPPIConfig: heading_error_scale_rad must be > 0")
        if self.speed_error_scale_mps <= 0:
            raise ValueError("MPPIConfig: speed_error_scale_mps must be > 0")
        if self.linear_command_change_scale_mps <= 0:
            raise ValueError("MPPIConfig: linear_command_change_scale_mps must be > 0")
        if self.angular_command_change_scale_radps <= 0:
            raise ValueError("MPPIConfig: angular_command_change_scale_radps must be > 0")
        if self.terminal_goal_error_scale_m <= 0:
            raise ValueError("MPPIConfig: terminal_goal_error_scale_m must be > 0")
        if self.path_distance_weight < 0:
            raise ValueError("MPPIConfig: path_distance_weight must be >= 0")
        if self.speed_weight < 0:
            raise ValueError("MPPIConfig: speed_weight must be >= 0")
        if self.heading_weight < 0:
            raise ValueError("MPPIConfig: heading_weight must be >= 0")
        if self.linear_smoothness_weight < 0:
            raise ValueError("MPPIConfig: linear_smoothness_weight must be >= 0")
        if self.angular_smoothness_weight < 0:
            raise ValueError("MPPIConfig: angular_smoothness_weight must be >= 0")
