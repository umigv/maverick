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

        self.get_logger().info(
            f"Loaded {len(self.obstacle_cells)} obstacles at {self.resolution_m} m/cell "
            f"({self.width_cells}x{self.height_cells} grid)"
        )

    def publish_ground_truth_map(self) -> None:
        if not self.obstacle_cells:
            return

        min_x = min(x for x, _ in self.obstacle_cells)
        min_y = min(y for _, y in self.obstacle_cells)
        max_x = max(x for x, _ in self.obstacle_cells)
        max_y = max(y for _, y in self.obstacle_cells)
        map_width_cells = max_x - min_x + 1
        map_height_cells = max_y - min_y + 1

        static_map = np.full((map_height_cells, map_width_cells), self.FREE, dtype=np.int8)
        for x, y in self.obstacle_cells:
            static_map[y - min_y, x - min_x] = self.OCCUPIED

        self.ground_truth_publisher.publish(
            OccupancyGrid(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.map_frame_id),
                info=MapMetaData(
                    resolution=self.resolution_m,
                    width=map_width_cells,
                    height=map_height_cells,
                    origin=Pose2d(Point2d(x=min_x, y=min_y) * self.resolution_m, Rotation2d(0.0)).to_ros(),
                ),
                data=static_map.flatten().tolist(),
            )
        )

    def publish_occupancy_grid(self) -> None:
        if self.robot_pose is None:
            return

        grid = np.full((self.height_cells, self.width_cells), self.FREE, dtype=np.int8)

        if len(self.obstacle_cells) != 0:
            for row in range(self.height_cells):
                for col in range(self.width_cells):
                    local = Point2d(
                        x=self.config.offset_x_m + (col + 0.5) * self.resolution_m,
                        y=self.config.offset_y_m + (row + 0.5) * self.resolution_m,
                    )
                    world = self.robot_pose.local_to_world(local)
                    ox = math.floor(world.x / self.resolution_m)
                    oy = math.floor(world.y / self.resolution_m)
                    if (ox, oy) in self.obstacle_cells:
                        grid[row, col] = self.OCCUPIED

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


def main() -> None:
    rclpy.init()
    node = OccupancyGridSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
