from __future__ import annotations

import json
import math

import numpy as np
import rclpy
import utils.config
import utils.qos
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Header
from utils.geometry import Point2d, Pose2d, Rotation2d

from .occupancy_grid_simulator_config import OccupancyGridSimulatorConfig


class OccupancyGridSimulator(Node):
    OCCUPIED = 100
    FREE = 0
    EPSILON = 1e-12

    def __init__(self) -> None:
        super().__init__("occupancy_grid_simulator")
        self.config: OccupancyGridSimulatorConfig = utils.config.load(self, OccupancyGridSimulatorConfig)

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, "occupancy_grid", 10)
        self.ground_truth_publisher = self.create_publisher(
            OccupancyGrid,
            "occupancy_grid/ground_truth",
            qos_profile=utils.qos.LATCHED,
        )
        self.ground_truth_lane_lines_publisher = self.create_publisher(
            OccupancyGrid,
            "occupancy_grid/ground_truth/lane_lines",
            qos_profile=utils.qos.LATCHED,
        )
        self.ground_truth_obstacles_publisher = self.create_publisher(
            OccupancyGrid,
            "occupancy_grid/ground_truth/obstacles",
            qos_profile=utils.qos.LATCHED,
        )

        self.robot_pose: Pose2d | None = None

        self.load_obstacles_and_configure_grid()
        self.publish_ground_truth_map()
        self.create_timer(self.config.publish_period_s, self.publish_occupancy_grid)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.map_frame_id:
            self.get_logger().warn(f"Dropping odometry: frame '{msg.header.frame_id}' != '{self.config.map_frame_id}'")
            return
        if msg.child_frame_id != self.config.ground_truth_base_frame_id:
            self.get_logger().warn(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != '{self.config.ground_truth_base_frame_id}'"
            )
            return

        self.robot_pose = Pose2d.from_ros(msg.pose.pose)

    def load_obstacles_and_configure_grid(self) -> None:
        try:
            with open(self.config.map_file_path) as file:
                data = json.load(file)
        except FileNotFoundError:
            self.get_logger().fatal(f"Map file not found: '{self.config.map_file_path}'")
            raise

        self.resolution_m: float = data["resolution_m"]
        self.width_cells: int = math.ceil(self.config.width_m / self.resolution_m)
        self.height_cells: int = math.ceil(self.config.height_m / self.resolution_m)
        self.obstacle_cells: frozenset[tuple[int, int]] = frozenset((x, y) for x, y in data["obstacles"])
        self.lane_line_cells: frozenset[tuple[int, int]] = frozenset((x, y) for x, y in data.get("lane_lines", []))

        self.obstacle_cell_keys = np.array([self.hash_cells(x, y) for x, y in self.obstacle_cells], dtype=np.int64)
        self.lane_line_cell_keys = np.array([self.hash_cells(x, y) for x, y in self.lane_line_cells], dtype=np.int64)

        self.get_logger().info(
            f"Loaded {len(self.obstacle_cells)} obstacle cells, {len(self.lane_line_cells)} lane line cells "
            f"at {self.resolution_m} m/cell ({self.width_cells}x{self.height_cells} grid)"
        )

    def _build_ground_truth_grid(self, cells: frozenset[tuple[int, int]]) -> OccupancyGrid | None:
        if not cells:
            return None

        xs, ys = zip(*cells, strict=True)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        map_width_cells = max_x - min_x + 1
        map_height_cells = max_y - min_y + 1

        grid = np.full((map_height_cells, map_width_cells), self.FREE, dtype=np.int8)
        for x, y in cells:
            grid[y - min_y, x - min_x] = self.OCCUPIED

        return OccupancyGrid(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.map_frame_id),
            info=MapMetaData(
                resolution=self.resolution_m,
                width=map_width_cells,
                height=map_height_cells,
                origin=Pose2d(Point2d(x=min_x, y=min_y) * self.resolution_m, Rotation2d(0.0)).to_ros(),
            ),
            data=grid.flatten().tolist(),
        )

    def publish_ground_truth_map(self) -> None:
        merged = self._build_ground_truth_grid(self.obstacle_cells | self.lane_line_cells)
        lane_lines = self._build_ground_truth_grid(self.lane_line_cells)
        obstacles = self._build_ground_truth_grid(self.obstacle_cells)

        if merged is not None:
            self.ground_truth_publisher.publish(merged)
        if lane_lines is not None:
            self.ground_truth_lane_lines_publisher.publish(lane_lines)
        if obstacles is not None:
            self.ground_truth_obstacles_publisher.publish(obstacles)

    def publish_occupancy_grid(self) -> None:
        if self.robot_pose is None:
            return

        grid, lane_line_mask = self.build_cell_maps(self.robot_pose)
        self.apply_raycasting(grid)
        grid[lane_line_mask] = self.OCCUPIED

        self.occupancy_grid_publisher.publish(
            OccupancyGrid(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.base_frame_id),
                info=MapMetaData(
                    resolution=self.resolution_m,
                    width=self.width_cells,
                    height=self.height_cells,
                    origin=Pose2d(
                        Point2d(x=self.config.offset_x_m, y=self.config.offset_y_m), Rotation2d(0.0)
                    ).to_ros(),
                ),
                data=grid.flatten().tolist(),
            )
        )

    @staticmethod
    def hash_cells(x: int | np.ndarray, y: int | np.ndarray) -> np.int64 | np.ndarray:
        """Encodes (x, y) as a single int64 for use with np.isin."""
        # Collisions require two y-values to differ by 2^32 (~4B cells), which is impossible in practice.
        return x * np.int64(1 << 32) + y

    def build_cell_maps(self, robot_pose: Pose2d) -> tuple[np.ndarray, np.ndarray]:
        """Returns (obstacle_grid, lane_line_mask) by mapping local cells to world coordinates."""
        local_x, local_y = np.meshgrid(
            self.config.offset_x_m + (np.arange(self.width_cells) + 0.5) * self.resolution_m,
            self.config.offset_y_m + (np.arange(self.height_cells) + 0.5) * self.resolution_m,
        )

        cos_yaw = robot_pose.rotation.cos
        sin_yaw = robot_pose.rotation.sin
        world_x = robot_pose.point.x + local_x * cos_yaw - local_y * sin_yaw
        world_y = robot_pose.point.y + local_x * sin_yaw + local_y * cos_yaw

        cell_x = np.floor(world_x / self.resolution_m).astype(np.int64)
        cell_y = np.floor(world_y / self.resolution_m).astype(np.int64)
        cell_keys = self.hash_cells(cell_x, cell_y)

        is_obstacle = np.isin(cell_keys, self.obstacle_cell_keys)
        is_lane_line = np.isin(cell_keys, self.lane_line_cell_keys)
        obstacle_grid = np.where(is_obstacle, self.OCCUPIED, self.FREE).astype(np.int8)
        return obstacle_grid, is_lane_line

    @staticmethod
    def slab_interval(
        delta: np.ndarray,
        robot_coord: float,
        obstacle_coord: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Slab method: returns (t_enter, t_exit) - the parametric interval on the ray robot + t*delta that passes through
        the 1-D slab [obstacle, obstacle+1). Handles parallel rays (|delta| ≈ 0): open interval if robot is inside the
        slab, empty interval otherwise.
        """
        parallel = np.abs(delta) <= OccupancyGridSimulator.EPSILON
        inside = (robot_coord >= obstacle_coord) & (robot_coord < obstacle_coord + 1.0)

        safe_delta = np.where(parallel, 1.0, delta)
        t_low = (obstacle_coord - robot_coord) / safe_delta
        t_high = (obstacle_coord + 1.0 - robot_coord) / safe_delta

        t_enter = np.where(parallel, np.where(inside, -np.inf, np.inf), np.minimum(t_low, t_high))
        t_exit = np.where(parallel, np.where(inside, np.inf, -np.inf), np.maximum(t_low, t_high))
        return t_enter, t_exit

    def apply_raycasting(self, obstacle_grid: np.ndarray) -> None:
        """Mark all cells behind obstacles (from the robot's perspective) as OCCUPIED."""
        obstacle_rows, obstacle_cols = np.where(obstacle_grid == self.OCCUPIED)
        if len(obstacle_rows) == 0:
            return

        robot_col = -self.config.offset_x_m / self.resolution_m - 0.5
        robot_row = -self.config.offset_y_m / self.resolution_m - 0.5

        # Axes: col → (1, W, 1), row → (H, 1, 1), obstacles → (1, 1, N); broadcast to (H, W, N).
        delta_col = np.arange(self.width_cells)[np.newaxis, :, np.newaxis] + 0.5 - robot_col
        delta_row = np.arange(self.height_cells)[:, np.newaxis, np.newaxis] + 0.5 - robot_row
        obs_col = obstacle_cols[np.newaxis, np.newaxis, :].astype(np.float64)
        obs_row = obstacle_rows[np.newaxis, np.newaxis, :].astype(np.float64)

        t_enter_col, t_exit_col = self.slab_interval(delta_col, robot_col, obs_col)
        t_enter_row, t_exit_row = self.slab_interval(delta_row, robot_row, obs_row)

        t_enter = np.maximum(t_enter_col, t_enter_row)  # (H, W, N)
        t_exit = np.minimum(t_exit_col, t_exit_row)  # (H, W, N)

        # Cell is occluded if any obstacle intersects the ray at t ∈ (0, 1).
        blocking = (t_enter < 1.0 - self.EPSILON) & (t_exit > self.EPSILON) & (t_enter < t_exit)
        obstacle_grid[np.any(blocking, axis=2)] = self.OCCUPIED


def main() -> None:
    rclpy.init()
    node = OccupancyGridSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
