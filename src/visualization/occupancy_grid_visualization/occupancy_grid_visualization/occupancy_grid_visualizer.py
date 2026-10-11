from math import isfinite

import numpy as np
import utils.lifecycle
from foxglove_msgs.msg import PackedElementField, VoxelGrid
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class FoxgloveOccupancyGrid(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_grid_voxel_visualization")
        self.voxel_height = float(self.declare_parameter("voxel_height", 0.1).value)
        if not isfinite(self.voxel_height) or self.voxel_height <= 0.0:
            raise ValueError("voxel_height must be finite and greater than zero")

        self.create_subscription(OccupancyGrid, "occupancy_grid", self.publish_foxglove_voxel_occupancy_grid, 10)

        self.voxel_occupancy_grid_publisher = self.create_publisher(VoxelGrid, "occupancy_grid/voxels", 10)
        self.get_logger().info(
            f"Converting {self.resolve_topic_name('occupancy_grid')} to "
            f"{self.voxel_occupancy_grid_publisher.topic_name}; voxel height={self.voxel_height:.2f} m"
        )

    def publish_foxglove_voxel_occupancy_grid(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        if (
            width == 0
            or height == 0
            or not isfinite(msg.info.resolution)
            or msg.info.resolution <= 0.0
            or len(msg.data) != width * height
        ):
            self.get_logger().warning(
                f"Skipping invalid grid: {width}x{height}, resolution={msg.info.resolution}, "
                f"data length={len(msg.data)}",
                throttle_duration_sec=5.0,
            )
            return

        self.voxel_occupancy_grid_publisher.publish(
            VoxelGrid(
                timestamp=msg.header.stamp,
                frame_id=msg.header.frame_id,
                pose=msg.info.origin,
                row_count=msg.info.height,
                column_count=msg.info.width,
                cell_size=Vector3(x=msg.info.resolution, y=msg.info.resolution, z=self.voxel_height),
                cell_stride=1,
                row_stride=width * 1,
                slice_stride=height * width * 1,
                fields=[PackedElementField(name="occupancy", offset=0, type=PackedElementField.INT8)],
                data=np.array(msg.data, dtype=np.int8).tobytes(),
            )
        )
        self.get_logger().info(
            f"Published voxel grid: {width}x{height}, resolution={msg.info.resolution:.3f} m, "
            f"frame={msg.header.frame_id!r}",
            throttle_duration_sec=5.0,
        )


def main() -> None:
    utils.lifecycle.run_node(FoxgloveOccupancyGrid)
