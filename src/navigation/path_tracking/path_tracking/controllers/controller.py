from typing import Literal, Protocol

from geometry_msgs.msg import Twist
from utils.geometry import Path2d, Pose2d

Algorithm = Literal["pure_pursuit", "stanley"]


class Controller(Protocol):
    def set_path(self, path: Path2d) -> None: ...
    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None: ...
