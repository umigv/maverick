import rclpy
import utils.config
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
    Transform,
    TransformStamped,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from utils.geometry import Point2d, Rotation2d

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)

        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self.config.publish_period_s, self.publish_odom)

        self.position = Point2d(x=0.0, y=0.0)
        self.rotation = Rotation2d(angle=0.0)
        self.prev_time: Time | None = None
        self.twist = TwistWithCovariance()

        # Row-major 6x6 covariance [x, y, z, roll, pitch, yaw]
        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0] = self.config.pose_x_variance_m2
        self.pose_covariance[7] = self.config.pose_y_variance_m2
        self.pose_covariance[35] = self.config.pose_yaw_variance_rad2

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.config.base_frame_id:
            self.get_logger().warn(f"Dropping enc_vel: frame '{msg.header.frame_id}' != '{self.config.base_frame_id}'")
            return

        current_time = Time.from_msg(msg.header.stamp)
        try:
            if self.prev_time is None:
                return

            dt = (current_time - self.prev_time).nanoseconds * 1e-9
            if dt <= 0.0 or dt > self.config.max_dt_s:
                return

            # Use midpoint method for higher numerical accuracy: https://en.wikipedia.org/wiki/Midpoint_method
            linear_vel = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z
            mid_rotation = self.rotation + Rotation2d(0.5 * angular_vel * dt)

            self.position += Point2d(x=linear_vel * dt, y=0.0).rotate_by(mid_rotation)
            self.rotation = mid_rotation + Rotation2d(0.5 * angular_vel * dt)
        finally:
            self.twist = msg.twist
            self.prev_time = current_time

    def publish_odom(self) -> None:
        now = self.get_clock().now()

        if self.prev_time is not None:
            age_s = (now - self.prev_time).nanoseconds * 1e-9
            twist = self.twist if age_s <= self.config.max_dt_s else TwistWithCovariance()
        else:
            twist = TwistWithCovariance()

        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=now.to_msg(), frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.position.x, y=self.position.y, z=0.0),
                    rotation=self.rotation.to_ros(),
                ),
            )
        )

        self.odom_publisher.publish(
            Odometry(
                header=Header(stamp=now.to_msg(), frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=self.position.to_ros(),
                        orientation=self.rotation.to_ros(),
                    ),
                    covariance=self.pose_covariance,
                ),
                twist=twist,
            )
        )


def main() -> None:
    rclpy.init()
    node = EncOdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
