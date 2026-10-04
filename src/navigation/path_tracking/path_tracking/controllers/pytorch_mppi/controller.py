import torch
from geometry_msgs.msg import Twist, Vector3
from pytorch_mppi import MPPI
from rclpy.impl.rcutils_logger import RcutilsLogger
from utils.geometry import Path2d, Pose2d

from .config import MPPIConfig


def wrap_angle(angle: torch.Tensor) -> torch.Tensor:
    """Wrap angles to [-pi, pi] so 359 degrees and -1 degree count as the same direction."""
    return torch.atan2(torch.sin(angle), torch.cos(angle))


class MPPIController:
    def __init__(self, config: MPPIConfig, logger: RcutilsLogger) -> None:
        self.config = config
        self.logger = logger
        self.path: Path2d | None = None
        self.progress_index: float = 0.0

        # Rebuilt every control step by update_lookahead(), read by the cost functions.
        self.lookahead_point = torch.zeros(2)
        self.lookahead_xy = torch.zeros(config.num_lookahead_poses, 2)
        self.lookahead_theta = torch.zeros(config.num_lookahead_poses)
        self.last_action = torch.zeros(2)  # last command actually sent, for the smoothing cost

        self.mppi = MPPI(
            self.dynamics,
            self.running_cost,
            nx=3,  # state = x, y, heading
            noise_sigma=torch.diag(torch.tensor([config.linear_noise_std_mps, config.angular_noise_std_radps])) ** 2,
            num_samples=config.num_samples,
            horizon=config.horizon_steps,
            lambda_=config.temperature,
            u_min=torch.tensor([0.0, -config.max_angular_speed_radps]),
            u_max=torch.tensor([config.max_linear_speed_mps, config.max_angular_speed_radps]),
            u_init=torch.tensor([config.max_linear_speed_mps / 2, 0.0]),  # CHECK: start mid-range, not at 0
            terminal_state_cost=self.terminal_cost,
        )

    def set_path(self, path: Path2d) -> None:
        self.path = path
        self.progress_index = 0.0
        self.last_action = torch.zeros(2)
        self.mppi.reset()  # CHECK: forget the plan made for the old path

    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None:
        if self.path is None:
            return None

        if pose.point.distance(self.path[-1]) < self.config.goal_tolerance_m:
            self.logger.info("Reached goal - stopping")
            self.path = None
            return Twist()

        lookahead_distance = self.config.max_linear_speed_mps * self.config.lookahead_time_s
        if not self.update_lookahead(pose, lookahead_distance):
            self.logger.warning("No valid lookahead found - stopping")
            return Twist()

        state = torch.tensor([pose.point.x, pose.point.y, pose.theta], dtype=torch.float32)  # CHECK heading name
        action = self.mppi.command(state)
        linear = min(float(action[0]), speed)
        angular = float(action[1])
        self.last_action = torch.tensor([linear, angular], dtype=torch.float32)
        return Twist(linear=Vector3(x=linear), angular=Vector3(z=angular))

    def update_lookahead(self, pose: Pose2d, lookahead_distance: float) -> bool:
        """Find the lookahead point and the lookahead poses leading up to it."""
        assert self.path is not None
        projection_index = self.path.project(pose.point, int(self.progress_index))
        if projection_index is None:
            return False
        self.progress_index = projection_index

        n = self.config.num_lookahead_poses
        # k = 0 is the robot's own spot on the path; advance() clamps at the last waypoint
        points = [self.path.advance(projection_index, lookahead_distance * k / n) for k in range(n + 1)]

        xy = torch.tensor([[p.x, p.y] for p in points], dtype=torch.float32)
        segments = xy[1:] - xy[:-1]
        theta = torch.atan2(segments[:, 1], segments[:, 0])
        for i in range(n):  # near the path's end, points can coincide; reuse the previous heading
            if segments[i].norm() < 1e-6:
                theta[i] = theta[i - 1] if i > 0 else pose.theta  # CHECK heading name

        self.lookahead_xy = xy[1:]
        self.lookahead_theta = theta
        self.lookahead_point = xy[-1]
        return True

    # --- the functions the library calls many times per control step ---

    def dynamics(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Where does each imagined robot end up after one tick?"""
        x, y, th = state[:, 0], state[:, 1], state[:, 2]
        v, w = action[:, 0], action[:, 1]
        dt = self.config.dt_s
        return torch.stack([x + v * torch.cos(th) * dt,
                            y + v * torch.sin(th) * dt,
                            th + w * dt], dim=1)

    def running_cost(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Alignment cost, charged at every step: stay near and parallel to the lookahead poses."""
        distances = torch.cdist(state[:, :2], self.lookahead_xy)  # (samples, num_lookahead_poses)
        nearest_distance, nearest_index = distances.min(dim=1)
        heading_error = wrap_angle(state[:, 2] - self.lookahead_theta[nearest_index])
        c = self.config
        return c.alignment_weight * nearest_distance**2 + c.heading_weight * heading_error**2

    def terminal_cost(self, states: torch.Tensor, actions: torch.Tensor) -> torch.Tensor:
        """Following and smoothing costs, charged once per whole imagined drive."""
        c = self.config
        horizon = states.shape[1]  # scale with the horizon so it keeps pace with the per-step cost
        following = ((states[:, -1, :2] - self.lookahead_point) ** 2).sum(dim=1)
        steps = torch.diff(actions, dim=1)                  # command changes within the drive
        first_step = actions[:, 0] - self.last_action       # change from what we just sent
        smoothing = (steps**2).sum(dim=(1, 2)) + (first_step**2).sum(dim=1)
        return c.following_weight * horizon * following + c.smoothing_weight * smoothing
