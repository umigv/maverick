import torch
from geometry_msgs.msg import Twist
from pytorch_mppi import MPPI
from rclpy.impl.rcutils_logger import RcutilsLogger
from utils.geometry import Path2d, Pose2d

from .config import MPPIConfig


class MPPIController:
    """Template for the MPPI path tracking controller."""

    def __init__(self, config: MPPIConfig, logger: RcutilsLogger) -> None:
        self.config = config
        self.logger = logger
        self.path: Path2d | None = None
        self.dtype = torch.float32

        noise_sigma = torch.tensor([[self.config.linear_noise_std**2, 0.0], [0.0, self.config.angular_noise_std**2]])

        # MPPI also has a smoothing variant, consider using
        # instead of adding it in cost function ourselves?

        self.ctrl = MPPI(
            self.dynamics,
            self.running_cost,
            3,
            noise_sigma,
            num_samples=self.config.num_samples,
            horizon=self.config.steps,
            lambda_=self.config.temperature,
            device="cpu",  # change later?
            u_min=torch.tensor([self.config.min_speed_mps, -self.config.max_angular_speed_radps]),
            u_max=torch.tensor([self.config.max_speed_mps, self.config.max_angular_speed_radps]),
        )

    def set_path(self, path: Path2d) -> None:
        points = torch.tensor(
            [[p.x, p.y] for p in path],
            dtype=self.dtype,
            device="cpu",  # change later?
        )

        starts = points[:-1]
        vectors = points[1:] - points[:-1]
        lengths = torch.linalg.vector_norm(vectors, dim=1)

        valid = lengths > 1e-6

        if not valid.any():
            self.logger.error("Path has no valid segments")
            return

        self.segment_starts = starts[valid]
        self.segment_vectors = vectors[valid]
        self.segment_lengths = lengths[valid]
        self.segment_lengths_squared = self.segment_vectors.square().sum(dim=1)  # d dot d

        self.segment_end_distances = torch.cumsum(self.segment_lengths, dim=0)

        self.path = path
        self.ctrl.reset()

    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None:
        if self.path is None:
            return None

        if pose.point.distance(self.path[-1]) < self.config.goal_tolerance_m:
            self.logger.info("Reached goal - stopping")
            self.path = None
            return Twist()

        state = torch.tensor(
            [pose.point.x, pose.point.y, pose.rotation.angle],
            dtype=self.dtype,
            device="cpu",  # change later?
        )

        with torch.no_grad():
            action = self.ctrl.command(state)

        cmd = Twist()
        cmd.linear.x = float(action[0].item())
        cmd.angular.z = float(action[1].item())

        return cmd

    def dynamics(self, state: torch.Tensor, control: torch.Tensor) -> torch.Tensor:
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]

        linear_speed = control[:, 0]
        angular_speed = control[:, 1]

        dt = self.config.dt

        next_x = x + linear_speed * torch.cos(theta) * dt
        next_y = y + linear_speed * torch.sin(theta) * dt
        next_theta = theta + angular_speed * dt

        # wrap theta in range of -pi to pi
        next_theta = torch.atan2(torch.sin(next_theta), torch.cos(next_theta))

        return torch.stack([next_x, next_y, next_theta], dim=1)

    def running_cost(self, state: torch.Tensor, control: torch.Tensor) -> torch.Tensor:
        if self.path is None:
            raise ValueError("Path is not set")

        config = self.config

        positions = state[:, :2]
        headings = state[:, 2]

        closest_points, segment_indices = self.find_closest_point_on_path(positions)

        path_errors = torch.linalg.vector_norm(positions - closest_points, dim=1)

        segment_vectors = self.segment_vectors[segment_indices]
        desired_headings = torch.atan2(segment_vectors[:, 1], segment_vectors[:, 0])

        heading_diff = headings - desired_headings
        heading_err = torch.atan2(torch.sin(heading_diff), torch.cos(heading_diff))

        speed_errors = control[:, 0] - config.target_speed_mps

        cost: torch.Tensor = (
            config.path_distance_weight * (path_errors / config.path_error_scale_m).square()
            + config.heading_weight * (heading_err / config.heading_error_scale_rad).square()
            + config.speed_weight * (speed_errors / config.speed_error_scale_mps).square()
        )
        return cost

    def find_closest_point_on_path(self, point: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:

        if self.path is None:
            raise ValueError("Path is not set")

        starts = self.segment_starts
        vectors = self.segment_vectors

        # all the (p - a)
        offsets = point[:, None, :] - starts[None, :, :]

        # (p - a) dot d, where d is the segment vector
        numerators = (offsets * vectors[None, :, :]).sum(dim=-1)

        # d dot d
        denominators = self.segment_lengths_squared[None, :]

        # the case where the point that forms a right angle with the infinite line
        # of the segment is not on the segment.
        fraction_of_segment = (numerators / denominators).clamp(0.0, 1.0)

        # q = a + t * d, where a is segment start,
        # t is fraction of segment, d is segment vector
        # q ends up being closest point on the segment to the robot.
        candidates = starts[None, :, :] + fraction_of_segment[:, :, None] * vectors[None, :, :]

        distances_sq = (point[:, None, :] - candidates).square().sum(dim=-1)

        reversed_indices = distances_sq.flip(dims=[1]).argmin(dim=1)
        segment_indices = starts.shape[0] - 1 - reversed_indices

        batched_indices = torch.arange(point.shape[0], device=point.device)

        closest_points = candidates[batched_indices, segment_indices]

        return closest_points, segment_indices
