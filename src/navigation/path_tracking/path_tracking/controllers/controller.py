from typing import Literal, Protocol

from geometry_msgs.msg import Twist
from utils.geometry import Path2d, Pose2d, Rotation2d

Algorithm = Literal["pure_pursuit", "stanley", "differential_drive", "mppi"]


class Controller(Protocol):
    # Current path, or None once the goal is reached.
    path: Path2d | None
    # Heading the controller is steering toward on the last command. None if not applicable.
    target_heading: Rotation2d | None

    def set_path(self, path: Path2d) -> None: ...
    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None: ...
