from dataclasses import dataclass


@dataclass(frozen=True)
class PathSmoothingConfig:
    """Configuration for the path smoothing node.

    Attributes:
        chaikin_iterations: Number of Chaikin corner-cutting iterations applied to the path. Each iteration halves
            corner sharpness. Set to 0 to disable smoothing.
    """

    chaikin_iterations: int = 3

    def __post_init__(self) -> None:
        if self.chaikin_iterations < 0:
            raise ValueError("PathSmoothingConfig: chaikin_iterations must be >= 0.")
