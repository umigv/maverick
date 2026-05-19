from __future__ import annotations

import math
import random

import numpy as np
import rclpy
import utils.config
import utils.qos
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from utils.geometry import Pose2d, Rotation2d

from .ultrasonic_simulator_config import UltrasonicSimulatorConfig

_OCCUPIED = 100


class UltrasonicSimulator(Node):
    def __init__(self) -> None:
        super().__init__("ultrasonic_simulator")
        self.config: UltrasonicSimulatorConfig = utils.config.load(self, UltrasonicSimulatorConfig)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_pose: Pose2d | None = None

        # Grid state populated on first obstacle-map message (latched, so received once).
        self.grid_data: np.ndarray | None = None
        self.grid_resolution: float = 1.0
        self.grid_width: int = 0
        self.grid_height: int = 0
        self.grid_origin_x: float = 0.0
        self.grid_origin_y: float = 0.0

        # frame_id -> (tx_m, ty_m, yaw_rad) of sensor link in base_link frame; cached after first lookup.
        self.sensor_transforms: dict[str, tuple[float, float, float]] = {}

        self.create_subscription(Odometry, "odom", self._odom_callback, 10)
        self.create_subscription(
            OccupancyGrid,
            "occupancy_grid/ground_truth/obstacles",
            self._obstacle_callback,
            qos_profile=utils.qos.LATCHED,
        )

        self.range_publisher = self.create_publisher(Range, self.config.topic, 10)

        self.create_timer(self.config.publish_period_s, self._publish)

    def _odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.map_frame_id:
            self.get_logger().warn(
                f"Dropping odometry: frame_id '{msg.header.frame_id}' != '{self.config.map_frame_id}'"
            )
            return
        if msg.child_frame_id != self.config.ground_truth_base_frame_id:
            self.get_logger().warn(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != "
                f"'{self.config.ground_truth_base_frame_id}'"
            )
            return
        self.robot_pose = Pose2d.from_ros(msg.pose.pose)

    def _obstacle_callback(self, msg: OccupancyGrid) -> None:
        self.grid_resolution = msg.info.resolution
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.grid_origin_x = msg.info.origin.position.x
        self.grid_origin_y = msg.info.origin.position.y
        self.grid_data = np.array(msg.data, dtype=np.int8).reshape(self.grid_height, self.grid_width)

    def _lookup_sensor_transforms(self) -> bool:
        """Looks up and caches base_link → sensor_link for all sensors. Returns True when all cached."""
        if len(self.sensor_transforms) == len(self.config.frame_ids):
            return True

        for frame_id in self.config.frame_ids:
            if frame_id in self.sensor_transforms:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(self.config.base_frame_id, frame_id, Time())
                tx = tf.transform.translation.x
                ty = tf.transform.translation.y
                yaw = Rotation2d.from_ros(tf.transform.rotation).angle
                self.sensor_transforms[frame_id] = (tx, ty, yaw)
            except TransformException as e:
                self.get_logger().warn(
                    f"TF {self.config.base_frame_id}->{frame_id} unavailable, skipping ultrasonic: {e}"
                )

        return len(self.sensor_transforms) == len(self.config.frame_ids)

    def _publish(self) -> None:
        if self.robot_pose is None or self.grid_data is None:
            return
        if not self._lookup_sensor_transforms():
            return

        stamp = self.get_clock().now().to_msg()
        robot = self.robot_pose
        cos_r = robot.rotation.cos
        sin_r = robot.rotation.sin

        ranges = []
        for tx, ty, sensor_yaw_base in self.sensor_transforms.values():
            # Sensor origin in world frame: rotate base-frame offset by robot yaw.
            sx = robot.point.x + cos_r * tx - sin_r * ty
            sy = robot.point.y + sin_r * tx + cos_r * ty

            # Sensing direction: sensor local +x rotated into world frame.
            total_yaw = robot.rotation.angle + sensor_yaw_base
            dx = math.cos(total_yaw)
            dy = math.sin(total_yaw)

            raw_range = self._raycast(sx, sy, dx, dy)
            measured = raw_range + random.gauss(0.0, self.config.noise_std_m)
            measured = max(self.config.min_range_m, min(measured, self.config.max_range_m))
            ranges.append(measured)

        self.range_publisher.publish(
            Range(
                header=Header(stamp=stamp, frame_id=self.config.topic + "_link"),
                radiation_type=Range.ULTRASOUND,
                field_of_view=self.config.field_of_view_rad,
                min_range=self.config.min_range_m,
                max_range=self.config.max_range_m,
                range=min(ranges),
            )
        )

    def _raycast(self, sx: float, sy: float, dx: float, dy: float) -> float:
        """DDA raycast from (sx, sy) in direction (dx, dy). Returns distance in metres to the
        first occupied cell in the ground truth obstacle grid, or max_range if none found.

        t is tracked in metres along the ray (dx, dy must form a unit vector).
        """
        if self.grid_data is None:
            raise RuntimeError("UltrasonicSimulator: grid_data is None in _raycast")

        res = self.grid_resolution
        max_range = self.config.max_range_m

        # Sensor position in grid cell coordinates.
        fx = (sx - self.grid_origin_x) / res
        fy = (sy - self.grid_origin_y) / res
        ix = math.floor(fx)
        iy = math.floor(fy)

        # DDA: t_delta = metres to traverse one full cell in each axis.
        # t_max   = metres to first cell boundary crossing in each axis.
        if abs(dx) < 1e-9:
            step_x, t_max_x, t_delta_x = 0, math.inf, math.inf
        else:
            step_x = 1 if dx > 0 else -1
            t_delta_x = abs(res / dx)
            boundary_x = ((ix + 1) if dx > 0 else ix) * res + self.grid_origin_x
            t_max_x = abs((boundary_x - sx) / dx)

        if abs(dy) < 1e-9:
            step_y, t_max_y, t_delta_y = 0, math.inf, math.inf
        else:
            step_y = 1 if dy > 0 else -1
            t_delta_y = abs(res / dy)
            boundary_y = ((iy + 1) if dy > 0 else iy) * res + self.grid_origin_y
            t_max_y = abs((boundary_y - sy) / dy)

        # t = distance at which the ray entered the current cell (0 for the starting cell).
        t = 0.0
        while t <= max_range:
            if 0 <= ix < self.grid_width and 0 <= iy < self.grid_height and self.grid_data[iy, ix] == _OCCUPIED:
                return t

            if t_max_x < t_max_y:
                t = t_max_x
                ix += step_x
                t_max_x += t_delta_x
            else:
                t = t_max_y
                iy += step_y
                t_max_y += t_delta_y

        return max_range


def main() -> None:
    rclpy.init()
    node = UltrasonicSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
