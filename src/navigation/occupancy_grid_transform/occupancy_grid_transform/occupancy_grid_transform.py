import numpy as np
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transform handlers
import utils.config
import utils.lifecycle
import utils.qos
from geometry_msgs.msg import PoseStamped
from maverick_msgs.msg import MissionState
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from .occupancy_grid_transform_config import OccupancyGridTransformConfig
from .occupancy_grid_transform_impl import add_border, inflate_grid


class OccupancyGridTransform(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_grid_transform")

        self.config: OccupancyGridTransformConfig = utils.config.load(self, OccupancyGridTransformConfig)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mission_state: MissionState | None = None

        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(MissionState, "mission_state", self.mission_state_callback, utils.qos.LATCHED)

        self.transformed_publisher = self.create_publisher(OccupancyGrid, "transformed_occupancy_grid", 10)
        self.inflated_publisher = self.create_publisher(OccupancyGrid, "inflated_occupancy_grid", 10)

    def mission_state_callback(self, msg: MissionState) -> None:
        self.mission_state = msg

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        # Drop the stamp so tf uses the latest available transform instead of looking up the exact time. If we copied
        # msg.header.stamp, a slightly future timestamp would cause tf to throw a "requested time is in the future"
        # error.
        #
        # The real fix would be adding a timeout to transform() and switching to a MultiThreadedExecutor so tf_listener
        # can update while we block, but that introduces thread-safety concerns. Using stamp=0 is acceptable because 5ms
        # of delay corresponds to only ~5mm of position error at 1 m/s.
        origin_raw = PoseStamped(header=Header(frame_id=msg.header.frame_id), pose=msg.info.origin)

        try:
            origin_transformed = self.tf_buffer.transform(origin_raw, self.config.frame_id).pose
        except Exception as e:
            self.get_logger().error(f"TF {msg.header.frame_id}->{self.config.frame_id} unavailable, skipping: {e}")
            return

        in_no_mans_land = self.mission_state is not None and self.mission_state.in_no_mans_land
        inflation_params = (
            self.config.no_mans_land_inflation_params if in_no_mans_land else self.config.inflation_params
        )

        grid = np.asarray(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        bordered_grid = add_border(grid)
        inflated_grid = inflate_grid(bordered_grid, inflation_params)
        height, width = grid.shape

        # We omit the stamp in the header to avoid TF extrapolation errors in rviz
        self.transformed_publisher.publish(
            OccupancyGrid(
                header=Header(frame_id=self.config.frame_id),
                info=MapMetaData(resolution=msg.info.resolution, width=width, height=height, origin=origin_transformed),
                data=bordered_grid.astype(np.int8).reshape(-1).tolist(),
            )
        )

        self.inflated_publisher.publish(
            OccupancyGrid(
                header=Header(frame_id=self.config.frame_id),
                info=MapMetaData(resolution=msg.info.resolution, width=width, height=height, origin=origin_transformed),
                data=inflated_grid.astype(np.int8).reshape(-1).tolist(),
            )
        )


def main() -> None:
    utils.lifecycle.run_node(OccupancyGridTransform)
