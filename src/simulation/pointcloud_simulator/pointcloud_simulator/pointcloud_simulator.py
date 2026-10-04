import json

import numpy as np
import utils.config
import utils.lifecycle
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from utils.geometry import Point2d, Pose2d

from .pointcloud_simulator_config import PointcloudSimulatorConfig
from .raycast import ObstacleMap, build_ray_directions, cast_rays, quaternion_from_rpy, rotation_matrix


class PointcloudSimulator(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_simulator")
        self.config: PointcloudSimulatorConfig = utils.config.load(self, PointcloudSimulatorConfig)

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, "pointcloud", 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.robot_pose: Pose2d | None = None
        self.random_generator = np.random.default_rng()

        self.mount_rotation = rotation_matrix(self.config.roll_rad, self.config.pitch_rad, self.config.yaw_rad)
        self.sensor_directions = build_ray_directions(
            self.config.horizontal_fov_rad,
            self.config.vertical_fov_rad,
            self.config.horizontal_resolution_rad,
            self.config.vertical_resolution_rad,
        )

        self.load_obstacle_map()
        self.publish_sensor_transform()
        self.create_timer(self.config.publish_period_s, self.publish_pointcloud)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.map_frame_id:
            self.get_logger().warning(
                f"Dropping odometry: frame '{msg.header.frame_id}' != '{self.config.map_frame_id}'"
            )
            return
        if msg.child_frame_id != self.config.ground_truth_base_frame_id:
            self.get_logger().warning(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != '{self.config.ground_truth_base_frame_id}'"
            )
            return

        self.robot_pose = Pose2d.from_ros(msg.pose.pose)

    def load_obstacle_map(self) -> None:
        try:
            with self.config.map_file_path.open() as file:
                data = json.load(file)
        except FileNotFoundError:
            self.get_logger().fatal(f"Map file not found: '{self.config.map_file_path}'")
            raise

        self.obstacle_map = ObstacleMap.from_cells(
            ((x, y) for x, y in data["obstacles"]), data["resolution_m"], self.config.obstacle_height_m
        )

        self.get_logger().info(
            f"Loaded {int(self.obstacle_map.occupied.sum())} obstacle cells at {data['resolution_m']} m/cell; "
            f"casting {len(self.sensor_directions)} rays from '{self.config.sensor_frame_id}'"
        )

    def publish_sensor_transform(self) -> None:
        x, y, z, w = quaternion_from_rpy(self.config.roll_rad, self.config.pitch_rad, self.config.yaw_rad)
        self.static_tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.base_frame_id),
                child_frame_id=self.config.sensor_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.config.offset_x_m, y=self.config.offset_y_m, z=self.config.offset_z_m),
                    rotation=Quaternion(x=x, y=y, z=z, w=w),
                ),
            )
        )

    def publish_pointcloud(self) -> None:
        if self.robot_pose is None:
            return

        sensor_position = self.robot_pose.local_to_world(Point2d(x=self.config.offset_x_m, y=self.config.offset_y_m))
        sensor_origin = np.array([sensor_position.x, sensor_position.y, self.config.offset_z_m])

        world_rotation = rotation_matrix(0.0, 0.0, self.robot_pose.rotation.angle) @ self.mount_rotation
        world_directions = self.sensor_directions @ world_rotation.T

        distances = cast_rays(sensor_origin, world_directions, self.obstacle_map, self.config.max_range_m)
        in_range = (distances >= self.config.min_range_m) & (distances <= self.config.max_range_m)
        distances = distances[in_range]
        if self.config.range_noise_std_m > 0:
            distances = distances + self.random_generator.normal(0.0, self.config.range_noise_std_m, len(distances))

        # Distance along a unit ray is frame-independent, so scaling the sensor-frame directions gives sensor-frame points.
        points = (self.sensor_directions[in_range] * distances[:, np.newaxis]).astype(np.float32)

        self.pointcloud_publisher.publish(
            point_cloud2.create_cloud_xyz32(
                Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.sensor_frame_id), points
            )
        )


def main() -> None:
    utils.lifecycle.run_node(PointcloudSimulator)
