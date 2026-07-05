from dataclasses import dataclass


@dataclass(frozen=True)
class PurePursuitConfig:
    """Configuration for the pure pursuit path tracking controller.

    Attributes:
        max_angular_speed_radps: Maximum angular velocity command (rad/s).
            Commands are scaled down to stay within this limit.
        base_lookahead_distance_m: Lookahead distance when stationary (m). Added to the speed-dependent term.
        min_lookahead_distance_m: Minimum clamped lookahead distance (m).
        max_lookahead_distance_m: Maximum clamped lookahead distance (m).
        lookahead_speed_gain: Gain applied to current speed when computing adaptive lookahead distance.
        linear_speed_gain: Gain applied to lookahead distance to produce the linear velocity command.
    """

    max_angular_speed_radps: float
    base_lookahead_distance_m: float
    min_lookahead_distance_m: float
    max_lookahead_distance_m: float
    lookahead_speed_gain: float
    linear_speed_gain: float

    def __post_init__(self) -> None:
        if self.max_angular_speed_radps <= 0:
            raise ValueError("PurePursuitConfig: max_angular_speed_radps must be > 0")
        if self.min_lookahead_distance_m <= 0:
            raise ValueError("PurePursuitConfig: min_lookahead_distance_m must be > 0")
        if self.max_lookahead_distance_m <= 0:
            raise ValueError("PurePursuitConfig: max_lookahead_distance_m must be > 0")
        if self.min_lookahead_distance_m > self.max_lookahead_distance_m:
            raise ValueError("PurePursuitConfig: min_lookahead_distance_m must be <= max_lookahead_distance_m")
        if self.linear_speed_gain <= 0:
            raise ValueError("PurePursuitConfig: linear_speed_gain must be > 0")
