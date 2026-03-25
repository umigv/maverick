import numpy as np
import rclpy
from foxglove_msgs.msg import PackedElementField, VoxelGrid
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class FoxgloveOccupancyGrid(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_grid_simulator")

        self.create_subscription(OccupancyGrid, "occupancy_grid", self.publish_foxglove_voxel_occupancy_grid, 10)

        self.voxel_occupancy_grid_publisher = self.create_publisher(VoxelGrid, "occupancy_grid/voxels", 10)

    def publish_foxglove_voxel_occupancy_grid(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height

        self.voxel_occupancy_grid_publisher.publish(
            VoxelGrid(
                timestamp=msg.header.stamp,
                frame_id=msg.header.frame_id,
                pose=msg.info.origin,
                row_count=msg.info.height,
                column_count=msg.info.width,
                cell_size=Vector3(x=msg.info.resolution, y=msg.info.resolution, z=1.0),
                cell_stride=1,
                row_stride=width * 1,
                slice_stride=height * width * 1,
                fields=[PackedElementField(name="occupancy", offset=0, type=PackedElementField.INT8)],
                data=np.array(msg.data, dtype=np.int8).tobytes(),
            )
        )


def main() -> None:
    rclpy.init()
    node = FoxgloveOccupancyGrid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
