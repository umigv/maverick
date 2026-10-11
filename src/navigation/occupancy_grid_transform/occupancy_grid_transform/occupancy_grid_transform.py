import numpy as np
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transform handlers
import utils.config
import utils.lifecycle
import utils.qos
from geometry_msgs.msg import Pose, PoseStamped
from maverick_msgs.msg import MissionState
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

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
        try:
            origin_transformed, placed_at_stamp = transform_grid_origin(self.tf_buffer, msg, self.config.frame_id)
        except TransformException as e:
            self.get_logger().error(f"TF {msg.header.frame_id}->{self.config.frame_id} unavailable, skipping: {e}")
            return
        if not placed_at_stamp:
            self.get_logger().warning(
                "No transform near the grid's stamp, placing it with the latest transform instead",
                throttle_duration_sec=5.0,
            )

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


# A grid stamped at most this far ahead of the latest transform is placed with the latest one without counting as a
# fallback: it is ordinary jitter between the grid producer and odometry, not a missing or stale stamp (s).
FUTURE_STAMP_TOLERANCE_S = 0.05


def transform_grid_origin(tf_buffer: Buffer, grid: OccupancyGrid, target_frame: str) -> tuple[Pose, bool]:
    """Return the grid origin in target_frame and whether it was placed at (or just after) the grid's stamp.

    Grids can arrive hundreds of milliseconds after capture. Placing one with the latest transform shifts its obstacles by
    however far the robot moved since the capture (measured in simulation: 28-37 cm median, up to 91 cm, 3 m ahead), so
    the transform at the stamp is used. Without blocking for it: if it is not available, the latest transform is used
    instead, as before, and the second value is False unless the stamp is at most FUTURE_STAMP_TOLERANCE_S ahead of
    the latest transform. A zero stamp also means latest.
    """
    origin = PoseStamped(header=Header(frame_id=grid.header.frame_id, stamp=grid.header.stamp), pose=grid.info.origin)
    try:
        return tf_buffer.transform(origin, target_frame).pose, True
    except TransformException:
        origin.header.stamp = Header().stamp
        latest = tf_buffer.lookup_transform(target_frame, grid.header.frame_id, Time())
        ahead_s = (Time.from_msg(grid.header.stamp) - Time.from_msg(latest.header.stamp)).nanoseconds * 1e-9
        return tf_buffer.transform(origin, target_frame).pose, 0.0 <= ahead_s <= FUTURE_STAMP_TOLERANCE_S


def main() -> None:
    utils.lifecycle.run_node(OccupancyGridTransform)
