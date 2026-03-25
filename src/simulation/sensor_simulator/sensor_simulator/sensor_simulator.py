import math
import random

import rclpy
import utils.config
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from pyproj import Transformer
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from utils.geometry import Point2d, Rotation2d

from .sensor_simulator_config import SensorSimulatorConfig


class SensorSimulator(Node):
    def __init__(self) -> None:
        super().__init__("sensor_simulator")
        self.config: SensorSimulatorConfig = utils.config.load(self, SensorSimulatorConfig)

        # Converts local ENU in meters first into ECEF (inverse topocentric) then into GPS longitude / latitude (inverse
        # cartesian). Origin is defined in the configuration
        lat, lon, alt = self.config.datum
        enu_to_lla = f"""
            +proj=pipeline
            +step +inv +proj=topocentric
                +lat_0={lat}
                +lon_0={lon}
                +h_0={alt}
                +ellps=WGS84
            +step +inv +proj=cart +ellps=WGS84
        """

        self.from_enu = Transformer.from_pipeline(enu_to_lla)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_rotation_offset: Rotation2d | None = None

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.enc_vel_publisher = self.create_publisher(TwistWithCovarianceStamped, "enc_vel", 10)
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.gps_publisher = self.create_publisher(NavSatFix, "gps", 10)
        self.ins_vel_publisher = self.create_publisher(TwistWithCovarianceStamped, "ins_vel", 10)
        self.ins_odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.ground_truth_publisher = self.create_publisher(Odometry, "odom/ground_truth", 10)

        self.position = Point2d(x=0.0, y=0.0)
        self.rotation = Rotation2d(self.config.initial_yaw_rad)
        self.cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()

        # Encoder measurement state
        self.enc_drift_scale_vx: float = 0.0
        self.enc_drift_scale_wz: float = 0.0

        self.enc_vx: float = 0.0
        self.enc_wz: float = 0.0

        # VN300 measurement state
        self.vn_drift_x: float = 0.0
        self.vn_drift_y: float = 0.0
        self.vn_drift_yaw: float = 0.0
        self.vn_drift_vx: float = 0.0
        self.vn_drift_wz: float = 0.0

        self.vn_x: float = 0.0
        self.vn_y: float = 0.0
        self.vn_yaw: float = self.rotation.angle
        self.vn_vx: float = 0.0
        self.vn_wz: float = 0.0

        self.create_timer(self.config.update_period_s, self.update)

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def update(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_cmd_time) > Duration(nanoseconds=int(self.config.cmd_vel_timeout_s * 1e9)):
            self.cmd_vel = Twist()

        dt = self.config.update_period_s
        self.position += Point2d(x=self.cmd_vel.linear.x * dt, y=0.0).rotate_by(self.rotation)
        self.rotation = Rotation2d(self.rotation.angle + self.cmd_vel.angular.z * dt)

        # fmt: off
        enc = self.config.enc_vel
        self.enc_drift_scale_vx = self.ou_update(self.enc_drift_scale_vx, enc.vx_drift_std, enc.drift_time_constant_s, dt)
        self.enc_drift_scale_wz = self.ou_update(self.enc_drift_scale_wz, enc.wz_drift_std, enc.drift_time_constant_s, dt)

        self.enc_vx = self.cmd_vel.linear.x * (1.0 + self.enc_drift_scale_vx) + random.gauss(0.0, enc.vx_noise_std_mps)
        self.enc_wz = self.cmd_vel.angular.z * (1.0 + self.enc_drift_scale_wz) + random.gauss(0.0, enc.wz_noise_std_radps)

        vn = self.config.vn300
        self.vn_drift_x = self.ou_update(self.vn_drift_x, vn.position_drift_std_m, vn.position_drift_time_constant_s, dt)
        self.vn_drift_y = self.ou_update(self.vn_drift_y, vn.position_drift_std_m, vn.position_drift_time_constant_s, dt)
        self.vn_drift_vx = self.ou_update(self.vn_drift_vx, vn.vx_drift_std_mps, vn.vel_drift_time_constant_s, dt)
        self.vn_drift_wz = self.ou_update(self.vn_drift_wz, vn.wz_drift_std_radps, vn.vel_drift_time_constant_s, dt)
        self.vn_drift_yaw = self.ou_update(self.vn_drift_yaw, vn.yaw_drift_std_rad, vn.yaw_drift_time_constant_s, dt)

        self.vn_x = self.position.x + self.vn_drift_x + random.gauss(0.0, vn.position_noise_std_m)
        self.vn_y = self.position.y + self.vn_drift_y + random.gauss(0.0, vn.position_noise_std_m)
        self.vn_yaw = self.rotation.angle + self.vn_drift_yaw + random.gauss(0.0, vn.yaw_noise_std_rad)
        self.vn_vx = self.cmd_vel.linear.x + self.vn_drift_vx + random.gauss(0.0, vn.vx_noise_std_mps)
        self.vn_wz = self.cmd_vel.angular.z + self.vn_drift_wz + random.gauss(0.0, vn.wz_noise_std_radps)
        # fmt: on

        stamp = now.to_msg()
        self.publish_ground_truth(stamp)
        self.publish_enc_vel(stamp)
        self.publish_imu(stamp)
        self.publish_ins_vel(stamp)
        self.publish_ins_odom(stamp)
        self.publish_gps(stamp)

    def publish_ground_truth(self, stamp) -> None:
        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=stamp, frame_id=self.config.map_frame_id),
                child_frame_id=self.config.ground_truth_base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.position.x, y=self.position.y, z=0.0),
                    rotation=self.rotation.to_ros(),
                ),
            )
        )

        self.ground_truth_publisher.publish(
            Odometry(
                header=Header(stamp=stamp, frame_id=self.config.map_frame_id),
                child_frame_id=self.config.ground_truth_base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=self.position.to_ros(),
                        orientation=self.rotation.to_ros(),
                    )
                ),
                twist=TwistWithCovariance(twist=self.cmd_vel),
            )
        )

    def publish_enc_vel(self, stamp) -> None:
        enc = self.config.enc_vel

        enc_vel_cov = [0.0] * 36
        enc_vel_cov[0] = enc.vx_noise_std_mps**2 + (self.enc_vx * enc.vx_drift_std) ** 2  # vx
        enc_vel_cov[7] = 1e-9  # vy
        enc_vel_cov[35] = enc.wz_noise_std_radps**2 + (self.enc_wz * enc.wz_drift_std) ** 2  # vyaw

        self.enc_vel_publisher.publish(
            TwistWithCovarianceStamped(
                header=Header(stamp=stamp, frame_id=self.config.base_frame_id),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=self.enc_vx),
                        angular=Vector3(z=self.enc_wz),
                    ),
                    covariance=enc_vel_cov,
                ),
            )
        )

    def publish_imu(self, stamp) -> None:
        vn = self.config.vn300

        self.get_imu_transform()
        if self.imu_rotation_offset is None:
            return

        imu_yaw = self.vn_yaw + self.imu_rotation_offset.angle

        # fmt: off
        orientation_cov = [
            vn.yaw_sigma2, 0.0,           0.0,
            0.0,           vn.yaw_sigma2, 0.0,
            0.0,           0.0,           vn.yaw_sigma2,
        ]

        angular_velocity_cov = [
            vn.wz_sigma2, 0.0,          0.0,
            0.0,          vn.wz_sigma2, 0.0,
            0.0,          0.0,          vn.wz_sigma2,
        ]

        # -1 for the first element means "no estimate" for sensor_msgs/Imu
        linear_acceleration_cov = [
            -1.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
        ]
        # fmt: on

        self.imu_publisher.publish(
            Imu(
                header=Header(stamp=stamp, frame_id=vn.imu_frame_id),
                orientation=Rotation2d(imu_yaw).to_ros(),
                orientation_covariance=orientation_cov,
                angular_velocity=Vector3(z=self.vn_wz),
                angular_velocity_covariance=angular_velocity_cov,
                linear_acceleration_covariance=linear_acceleration_cov,
            )
        )

    def publish_ins_vel(self, stamp) -> None:
        vn = self.config.vn300

        cov = [0.0] * 36
        cov[0] = vn.vx_sigma2  # vx
        cov[7] = 1e-9  # vy
        cov[35] = vn.wz_sigma2  # vyaw

        self.ins_vel_publisher.publish(
            TwistWithCovarianceStamped(
                header=Header(stamp=stamp, frame_id=self.config.vn300.ins_frame_id),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=self.vn_vx),
                        angular=Vector3(z=self.vn_wz),
                    ),
                    covariance=cov,
                ),
            )
        )

    def publish_ins_odom(self, stamp) -> None:
        vn = self.config.vn300

        pose_cov = [0.0] * 36
        pose_cov[0] = vn.position_sigma2  # x
        pose_cov[7] = vn.position_sigma2  # y
        pose_cov[14] = 1e-9  # z
        pose_cov[21] = 1e-9  # roll
        pose_cov[28] = 1e-9  # pitch
        pose_cov[35] = vn.yaw_sigma2  # yaw

        twist_cov = [0.0] * 36
        twist_cov[0] = vn.vx_sigma2  # vx
        twist_cov[7] = 1e-9  # vy
        twist_cov[35] = vn.wz_sigma2  # vyaw

        self.ins_odom_publisher.publish(
            Odometry(
                header=Header(stamp=stamp, frame_id=self.config.map_frame_id),
                child_frame_id=self.config.vn300.ins_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point2d(x=self.vn_x, y=self.vn_y).to_ros(),
                        orientation=Rotation2d(self.vn_yaw).to_ros(),
                    ),
                    covariance=pose_cov,
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=self.vn_vx),
                        angular=Vector3(z=self.vn_wz),
                    ),
                    covariance=twist_cov,
                ),
            )
        )

    def publish_gps(self, stamp) -> None:
        vn = self.config.vn300
        lon, lat, alt = self.from_enu.transform(self.vn_x, self.vn_y, 0.0)

        # fmt: off
        gps_cov = [
            vn.position_sigma2, 0.0,                0.0,
            0.0,                vn.position_sigma2, 0.0,
            0.0,                0.0,                vn.position_sigma2,
        ]
        # fmt: on

        self.gps_publisher.publish(
            NavSatFix(
                header=Header(stamp=stamp, frame_id=vn.ins_frame_id),
                status=NavSatStatus(status=NavSatStatus.STATUS_SBAS_FIX, service=NavSatStatus.SERVICE_GPS),
                latitude=lat,
                longitude=lon,
                altitude=alt,
                position_covariance=gps_cov,
                position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            )
        )

    def get_imu_transform(self):
        if self.imu_rotation_offset is not None:
            return

        vn = self.config.vn300

        try:
            self.imu_rotation_offset = Rotation2d.from_ros(
                self.tf_buffer.lookup_transform(self.config.base_frame_id, vn.imu_frame_id, Time()).transform.rotation
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.config.base_frame_id}->{vn.imu_frame_id} unavailable, skipping publishing IMU: {e}"
            )

    @staticmethod
    def ou_update(value: float, sigma: float, tau: float, dt: float) -> float:
        """Does one iteration of Ornstein-Uhlenbeck process.
        https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process

        Args:
            value: Current state x_t.
            sigma: Steady-state standard deviation. Units match value.
            tau: Mean-reversion time constant in seconds. Larger values produce slower-drifting, more correlated noise.
            dt: Elapsed time in seconds. If <= 0, value is returned unchanged.

        Returns:
            Updated state.
        """
        if dt <= 0.0:
            return value

        decay = math.exp(-dt / tau)
        innov_std = sigma * math.sqrt(max(0.0, 1.0 - decay**2))
        return value * decay + innov_std * random.gauss(0.0, 1.0)


def main() -> None:
    rclpy.init()
    node = SensorSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
