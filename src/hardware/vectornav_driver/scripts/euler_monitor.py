import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def euler_from_quaternion(x, y, z, w):
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuEulerNode(Node):
    def __init__(self, topic):
        super().__init__("imu_euler_node")
        self.get_logger().info(f"Reading IMU data from topic: {topic}")
        self.subscription = self.create_subscription(Imu, topic, self.imu_callback, 10)

    def imu_callback(self, msg):
        orientation = msg.orientation
        roll, pitch, yaw = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        self.get_logger().info(f"Roll: {roll_deg:7.2f}°  Pitch: {pitch_deg:7.2f}°  Yaw: {yaw_deg:7.2f}°")


def main():
    parser = argparse.ArgumentParser(description="Print IMU orientation as Euler angles")
    parser.add_argument("topic", nargs="?", default="vectornav/imu", help="IMU topic name")
    args = parser.parse_args()

    rclpy.init()
    node = ImuEulerNode(args.topic)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
