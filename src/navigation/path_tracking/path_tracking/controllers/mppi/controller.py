import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import Twist, Vector3
from rclpy.impl.rcutils_logger import RcutilsLogger
from utils.geometry import Path2d, Pose2d

from .config import MPPIConfig

FloatArray = npt.NDArray[np.float64]

# Extra arc length beyond the farthest reachable distance that rollouts are projected against (m).
WINDOW_MARGIN_M = 1.0
# How far past the distance a rollout has actually traveled its states may be matched along the path (m). Without this
# bound, a rollout that cuts across to a nearby later part of the path (the far leg of a hairpin around an obstacle) is
# matched to that part and rewarded for skipping everything in between. The margin covers a robot inside a tight
# curve, whose projection legitimately advances faster than it moves.
PROJECTION_ARC_MARGIN_M = 0.5


def wrap_angle(angle: npt.ArrayLike) -> FloatArray:
    """Wrap angles to [-pi, pi]."""
    return np.asarray(np.arctan2(np.sin(angle), np.cos(angle)), dtype=np.float64)


class MPPIController:
    """Sampling-based MPPI path tracker over a unicycle model.

    State is [x, y, yaw] and control is [v, omega]. Velocity, acceleration and wheel-speed limits are enforced on every
    sampled control (see limit_controls), so every rollout and their weighted average are feasible.
    """

    def __init__(self, config: MPPIConfig, control_period_s: float, logger: RcutilsLogger) -> None:
        self.config = config
        self.control_period_s = control_period_s
        self.logger = logger
        self.rng = np.random.default_rng(None if config.seed < 0 else config.seed)
        self.noise_std = np.array([config.linear_noise_std_mps, config.angular_noise_std_radps])
        self.max_speed_mps = min(config.max_linear_speed_mps, config.max_wheel_speed_mps)
        self.half_track_m = config.track_width_m / 2.0

        self.path: Path2d | None = None
        self.projection_index: int = 0
        # Path resampled to path_resolution_m, cached as arrays for vectorized projection
        self.points = np.zeros((0, 2))
        self.segments = np.zeros((0, 2))
        self.segment_headings = np.zeros(0)
        self.inv_segment_length_sq = np.zeros(0)
        self.cumulative_length = np.zeros(0)
        self.original_cumulative_length = np.zeros(0)

        # Unit-variance exploration noise kept between calls when config.reuse_noise is set; see exploration_noise()
        self.reused_noise: FloatArray | None = None
        self.next_noise_refresh = 0

        self.last_command = np.zeros(2)
        self.nominal = np.zeros((config.horizon_steps, 2))

    def set_path(self, path: Path2d) -> None:
        self.path = path
        self.projection_index = 0

        original = np.array([[p.x, p.y] for p in path])
        original_lengths = np.hypot(*np.diff(original, axis=0).T)
        self.original_cumulative_length = np.concatenate([[0.0], np.cumsum(original_lengths)])
        total_length = self.original_cumulative_length[-1]
        num_points = max(2, int(np.ceil(total_length / self.config.path_resolution_m)) + 1)
        arc = np.linspace(0.0, total_length, num_points)
        self.points = np.stack(
            [
                np.interp(arc, self.original_cumulative_length, original[:, 0]),
                np.interp(arc, self.original_cumulative_length, original[:, 1]),
            ],
            axis=1,
        )
        self.segments = np.diff(self.points, axis=0)
        segment_length_sq = np.sum(self.segments**2, axis=1)
        self.segment_headings = np.arctan2(self.segments[:, 1], self.segments[:, 0])
        self.inv_segment_length_sq = 1.0 / segment_length_sq
        self.cumulative_length = arc

        # Restart the nominal sequence from the last applied command so replanning does not violate rate limits
        self.nominal = np.tile(self.last_command, (self.config.horizon_steps, 1))

    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None:
        if self.path is None:
            return None

        if pose.point.distance(self.path[-1]) < self.config.goal_tolerance_m:
            self.logger.info("Reached goal - stopping")
            self.reset()
            return Twist()

        new_projection_index = self.path.project(pose.point, self.projection_index)
        if new_projection_index is None:
            self.logger.warning("MPPI: no valid projection segment - stopping")
            self.last_command = np.zeros(2)
            return Twist()

        if new_projection_index >= len(self.path) - 1:
            self.logger.info("Reached end of path - stopping")
            self.reset()
            return Twist()

        segment_index = int(new_projection_index)
        segment_length = (
            self.original_cumulative_length[segment_index + 1] - self.original_cumulative_length[segment_index]
        )
        start_arc_m = (
            self.original_cumulative_length[segment_index] + (new_projection_index - segment_index) * segment_length
        )
        self.projection_index = segment_index

        # The measured speed is unsigned (odometry speed magnitude), so only resynchronize downward: the robot is clearly
        # slower than the last command, because the command was capped or overridden downstream. Lowering v keeps the
        # stored command feasible.
        if self.last_command[0] - speed > self.config.speed_resync_threshold_mps:
            self.logger.debug(
                f"MPPI: measured speed {speed:.2f} m/s is below the last command {self.last_command[0]:.2f} m/s, "
                "resynchronizing",
                throttle_duration_sec=1.0,
            )
            self.last_command[0] = speed

        self.optimize(pose.point.x, pose.point.y, pose.rotation.angle, start_arc_m)

        dt = self.control_period_s
        max_dv, max_domega = self.config.max_linear_accel_mps2 * dt, self.config.max_angular_accel_radps2 * dt
        v, omega = self.limit_controls(
            self.nominal[0, 0], self.nominal[0, 1], self.last_command[0], self.last_command[1], max_dv, max_domega
        )
        self.last_command = np.array([float(v), float(omega)])

        # Warm start: advance the nominal sequence by one control period, holding the final control
        horizon_times = np.arange(self.config.horizon_steps) * self.config.timestep_s
        self.nominal = np.stack(
            [np.interp(horizon_times + dt, horizon_times, self.nominal[:, i]) for i in range(2)], axis=1
        )

        return Twist(linear=Vector3(x=self.last_command[0]), angular=Vector3(z=self.last_command[1]))

    def reset(self) -> None:
        self.path = None
        self.projection_index = 0
        self.last_command = np.zeros(2)
        self.nominal = np.zeros((self.config.horizon_steps, 2))

    def limit_controls(
        self,
        v: npt.ArrayLike,
        omega: npt.ArrayLike,
        prev_v: npt.ArrayLike,
        prev_omega: npt.ArrayLike,
        max_dv: float,
        max_domega: float,
    ) -> tuple[FloatArray, FloatArray]:
        """Project controls onto the feasible set given the previous control.

        Enforces 0 <= v <= max speed, |omega| <= max angular speed, per-step rate limits, and
        |v| + |omega| * track_width / 2 <= max wheel speed. Speed is capped first so that some omega within the rate
        limit always satisfies the wheel limit, which keeps the set non-empty whenever the previous control is feasible.
        """
        cfg = self.config
        prev_v = np.asarray(prev_v)
        prev_omega = np.asarray(prev_omega)
        min_reachable_omega = np.maximum(np.abs(prev_omega) - max_domega, 0.0)
        v_max = np.minimum(self.max_speed_mps, cfg.max_wheel_speed_mps - self.half_track_m * min_reachable_omega)
        v_out = np.clip(v, np.maximum(prev_v - max_dv, 0.0), np.minimum(prev_v + max_dv, v_max))
        omega_max = np.minimum(cfg.max_angular_speed_radps, (cfg.max_wheel_speed_mps - v_out) / self.half_track_m)
        omega_out = np.clip(
            omega, np.maximum(prev_omega - max_domega, -omega_max), np.minimum(prev_omega + max_domega, omega_max)
        )
        return np.asarray(v_out, dtype=np.float64), np.asarray(omega_out, dtype=np.float64)

    def optimize(self, x: float, y: float, yaw: float, start_arc_m: float) -> None:
        """Run one MPPI update of self.nominal from the given robot state."""
        cfg = self.config
        num_samples, horizon, dt = cfg.num_samples, cfg.horizon_steps, cfg.timestep_s

        noise = self.exploration_noise() * self.noise_std
        noise[0] = 0.0  # Always evaluate the nominal sequence itself
        sampled = self.nominal + noise

        controls = np.empty((num_samples, horizon, 2))
        xs = np.empty((num_samples, horizon))
        ys = np.empty((num_samples, horizon))
        yaws = np.empty((num_samples, horizon))
        x_k = np.full(num_samples, x)
        y_k = np.full(num_samples, y)
        yaw_k = np.full(num_samples, yaw)
        v_k = np.full(num_samples, self.last_command[0])
        omega_k = np.full(num_samples, self.last_command[1])
        max_dv, max_domega = cfg.max_linear_accel_mps2 * dt, cfg.max_angular_accel_radps2 * dt
        for t in range(horizon):
            v_k, omega_k = self.limit_controls(sampled[:, t, 0], sampled[:, t, 1], v_k, omega_k, max_dv, max_domega)
            x_k = x_k + v_k * np.cos(yaw_k) * dt
            y_k = y_k + v_k * np.sin(yaw_k) * dt
            yaw_k = yaw_k + omega_k * dt
            controls[:, t, 0], controls[:, t, 1] = v_k, omega_k
            xs[:, t], ys[:, t], yaws[:, t] = x_k, y_k, yaw_k

        costs = self.rollout_costs(xs, ys, yaws, controls, start_arc_m)
        weights = np.exp(-(costs - costs.min()) / cfg.temperature)
        weights /= weights.sum()
        # Every sampled sequence is feasible from the same initial control and the constraints are convex, so their
        # weighted average is feasible too
        self.nominal = np.einsum("k,ktc->tc", weights, controls)

    def exploration_noise(self) -> FloatArray:
        """Return unit-variance noise of shape (K, T, 2), fresh or reused according to config.reuse_noise.

        Reuse keeps the sample set between calls and replaces noise_refresh_pairs_per_call samples (and their mirrors)
        per call, round-robin, so the whole set renews every num_samples / (2 * noise_refresh_pairs_per_call) calls.
        Consecutive updates then see almost the same samples and the weighted average no longer jumps from call to call. The second half of the samples mirrors the first, so the
        samples sum to exactly zero and a kept set cannot bias the update in one direction.
        """
        cfg = self.config
        if not cfg.reuse_noise:
            return self.rng.standard_normal((cfg.num_samples, cfg.horizon_steps, 2))
        half = cfg.num_samples // 2
        if self.reused_noise is None:
            self.reused_noise = self.rng.standard_normal((half, cfg.horizon_steps, 2))
            self.reused_noise[0] = 0.0  # sample 0 is the unperturbed nominal; keep its mirror unperturbed too
        else:
            for _ in range(cfg.noise_refresh_pairs_per_call):
                self.next_noise_refresh = self.next_noise_refresh % (half - 1) + 1  # cycles 1 .. half - 1
                self.reused_noise[self.next_noise_refresh] = self.rng.standard_normal((cfg.horizon_steps, 2))
        noise = np.zeros((cfg.num_samples, cfg.horizon_steps, 2))
        noise[:half] = self.reused_noise
        noise[half : 2 * half] = -self.reused_noise
        return noise

    def rollout_costs(
        self, xs: FloatArray, ys: FloatArray, yaws: FloatArray, controls: FloatArray, start_arc_m: float
    ) -> FloatArray:
        """Return the total cost of each rollout. xs, ys, yaws are (K, T); controls is (K, T, 2)."""
        cfg = self.config
        reach_m = self.max_speed_mps * cfg.horizon_steps * cfg.timestep_s + WINDOW_MARGIN_M
        first = int(np.searchsorted(self.cumulative_length, start_arc_m, side="right")) - 1
        first = min(max(first, 0), len(self.segments) - 1)
        last = int(np.searchsorted(self.cumulative_length, start_arc_m + reach_m)) + 1
        last = min(max(last, first + 1), len(self.segments))

        traveled_m = np.cumsum(controls[:, :, 0], axis=1) * cfg.timestep_s
        max_arc_m = start_arc_m + traveled_m + PROJECTION_ARC_MARGIN_M
        distance_sq, segment, arc_m = self.project(xs, ys, first, last, max_arc_m)

        heading_error = wrap_angle(yaws - self.segment_headings[segment])
        remaining_m = np.maximum(self.cumulative_length[-1] - arc_m, 0.0)
        # Reference speed tapers so the robot can decelerate to zero at max_linear_accel_mps2 by the point where it stops:
        # goal_tolerance_m before the end of the path
        stopping_m = np.maximum(remaining_m - cfg.goal_tolerance_m, 0.0)
        reference_speed = np.minimum(cfg.target_speed_mps, np.sqrt(2.0 * cfg.max_linear_accel_mps2 * stopping_m))
        previous = np.concatenate(
            [np.broadcast_to(self.last_command, (controls.shape[0], 1, 2)), controls[:, :-1]], axis=1
        )
        rate = controls - previous

        running = (
            cfg.cross_track_weight * distance_sq
            + cfg.heading_weight * heading_error**2
            + cfg.speed_weight * (controls[:, :, 0] - reference_speed) ** 2
            + cfg.linear_rate_weight * rate[:, :, 0] ** 2
            + cfg.angular_rate_weight * rate[:, :, 1] ** 2
        )
        terminal = cfg.terminal_cross_track_weight * distance_sq[:, -1] - cfg.progress_weight * (
            arc_m[:, -1] - start_arc_m
        )
        return np.asarray(running.sum(axis=1) * cfg.timestep_s + terminal, dtype=np.float64)

    def project(
        self, xs: FloatArray, ys: FloatArray, first: int, last: int, max_arc_m: FloatArray
    ) -> tuple[FloatArray, npt.NDArray[np.intp], FloatArray]:
        """Project points onto resampled path segments [first, last), each only onto segments starting by its max_arc_m.

        Returns squared distance, segment index and arc length of each projection. Computed in float32 since this
        (K, T, window) step dominates the update time. Segment `first` contains the robot's own projection, so every
        point always has at least one eligible segment.
        """
        start = self.points[first:last].astype(np.float32)
        direction = self.segments[first:last].astype(np.float32)
        inv_length_sq = self.inv_segment_length_sq[first:last].astype(np.float32)
        rel_x = xs.astype(np.float32)[..., None] - start[:, 0]
        rel_y = ys.astype(np.float32)[..., None] - start[:, 1]
        t = np.clip((rel_x * direction[:, 0] + rel_y * direction[:, 1]) * inv_length_sq, 0.0, 1.0)
        err_x = rel_x - t * direction[:, 0]
        err_y = rel_y - t * direction[:, 1]
        distance_sq = err_x * err_x + err_y * err_y
        unreachable = self.cumulative_length[first:last].astype(np.float32) > max_arc_m.astype(np.float32)[..., None]
        np.putmask(distance_sq, unreachable, np.float32(np.inf))
        nearest = distance_sq.argmin(axis=-1)[..., None]
        best_distance_sq = np.take_along_axis(distance_sq, nearest, axis=-1)[..., 0].astype(np.float64)
        best_t = np.take_along_axis(t, nearest, axis=-1)[..., 0].astype(np.float64)
        segment = nearest[..., 0] + first
        arc_m = self.cumulative_length[segment] + best_t * (
            self.cumulative_length[segment + 1] - self.cumulative_length[segment]
        )
        return best_distance_sq, segment, arc_m
