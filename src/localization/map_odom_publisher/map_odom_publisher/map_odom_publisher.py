import PyKDL
import rclpy
import utils.config
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .map_odom_publisher_config import MapOdomPublisherConfig


def frame_from_odom(odom: Odometry) -> PyKDL.Frame:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w),
        PyKDL.Vector(p.x, p.y, p.z),
    )


def frame_from_transform(tf: TransformStamped) -> PyKDL.Frame:
    t = tf.transform.translation
    q = tf.transform.rotation
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w),
        PyKDL.Vector(t.x, t.y, t.z),
    )


class MapOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("map_odom_publisher")

        self.config = utils.config.load(self, MapOdomPublisherConfig)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom: Odometry | None = None

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # We publish at a lower rate because querying TF at odom rate had performance issues. The lower rate is not too
        # big of an issue since map to odom transform change is slow (drift driven) and the only consumer is
        # autonav_goal_selection which expects the TF at a max of 1hz and only looks up the latest available timestamp
        self.create_timer(self.config.publish_period_s, self.publish)

    def odom_callback(self, odom: Odometry) -> None:
        if odom.header.frame_id != self.config.map_frame_id:
            self.get_logger().error(
                f"Dropping odometry: frame_id '{odom.header.frame_id}' != map_frame_id '{self.config.map_frame_id}'"
            )
            return

        if odom.child_frame_id != self.config.base_frame_id:
            self.get_logger().error(
                f"Dropping odometry: child_frame_id '{odom.child_frame_id}' != base_frame_id '{self.config.base_frame_id}'"
            )
            return

        self.odom = odom

    def publish(self) -> None:
        if self.odom is None:
            return

        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.config.odom_frame_id,
                self.config.base_frame_id,
                Time(),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.config.odom_frame_id}->{self.config.base_frame_id} unavailable, skipping: {e}"
            )
            return

        T_map_odom = frame_from_odom(self.odom) * frame_from_transform(tf_odom_base).Inverse()
        p = T_map_odom.p
        q = T_map_odom.M.GetQuaternion()  # (x, y, z, w)

        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=self.odom.header.stamp, frame_id=self.config.map_frame_id),
                child_frame_id=self.config.odom_frame_id,
                transform=Transform(
                    translation=Vector3(x=p.x(), y=p.y(), z=p.z()),
                    rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                ),
            )
        )


def main() -> None:
    rclpy.init()
    node = MapOdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
