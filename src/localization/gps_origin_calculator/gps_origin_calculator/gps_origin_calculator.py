import json
import math
from statistics import median

import rclpy
import utils.config
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from .gps_origin_calculator_config import GpsOriginCalculatorConfig


class GpsOriginCalculator(Node):
    def __init__(self) -> None:
        super().__init__("gps_origin_calculator")

        self.config = utils.config.load(self, GpsOriginCalculatorConfig)

        self.gps_subscriber = self.create_subscription(NavSatFix, "gps", self.gps_callback, 10)
        self.get_logger().info(f"Subscribing to GPS data on topic: {self.gps_subscriber.topic_name}")

        self.samples: list[NavSatFix] = []
        self.done = False

        self.get_logger().info(
            f"Collecting GNSS samples for datum. Keep robot STILL.\n"
            f"Policy: min_samples={self.config.min_samples_required}, max_horizontal_stdev={self.config.max_horizontal_stdev_m}m,\n"
            f"        min_duration={self.config.min_sample_duration_s}s, max_duration={self.config.max_sample_duration_s}s\n"
        )

    def gps_callback(self, msg: NavSatFix) -> None:
        if self.done:
            return

        self.get_logger().debug(f"Received data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")

        if msg.status.status == msg.status.STATUS_NO_FIX:
            self.get_logger().debug("Dropping GPS msg with no fix")
            return

        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            self.get_logger().debug("Dropping GPS msg with unknown covariance type")
            return

        horizontal_stdev = math.sqrt(max(msg.position_covariance[0], msg.position_covariance[4]))
        if horizontal_stdev > self.config.max_horizontal_stdev_m:
            self.get_logger().debug(
                f"Dropping GPS msg with high horizontal stdev: {horizontal_stdev} > {self.config.max_horizontal_stdev_m}"
            )
            return

        self.samples.append(msg)

        time_elapsed = self.compute_time_elapsed()
        sufficient_samples_collected = (
            len(self.samples) >= self.config.min_samples_required and time_elapsed >= self.config.min_sample_duration_s
        )
        max_duration_reached = time_elapsed >= self.config.max_sample_duration_s

        if not sufficient_samples_collected and not max_duration_reached:
            return

        if sufficient_samples_collected:
            self.get_logger().info("Sufficient samples collected.")
        elif max_duration_reached:
            self.get_logger().info("Max sample duration reached.")

        self.done = True
        self.compute_and_print_origin()

        # We don't call rclpy.shutdown() here because it causes a deadlock in humble
        # https://github.com/ros2/rclpy/issues/1646
        raise SystemExit(0)

    def compute_and_print_origin(self) -> None:
        self.get_logger().info(f"Collected {len(self.samples)} samples:")
        latitude = median(sample.latitude for sample in self.samples)
        longitude = median(sample.longitude for sample in self.samples)
        altitude = median(sample.altitude for sample in self.samples)
        self.get_logger().info(f"Origin is lat={latitude:.8f}, lon={longitude:.8f}, alt={altitude:.3f}m")

        if self.config.output_file.name:
            self.write_origin_to_file(latitude, longitude, altitude)

    def write_origin_to_file(self, latitude: float, longitude: float, altitude: float) -> None:
        try:
            with self.config.output_file.open() as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            self.get_logger().warn(f"Could not read existing file {self.config.output_file}: {e}. Overwriting.")
            data = {}

        data["datum"] = {"latitude": latitude, "longitude": longitude, "altitude": altitude}

        self.config.output_file.parent.mkdir(parents=True, exist_ok=True)
        with self.config.output_file.open("w") as f:
            json.dump(data, f, indent=4)
            f.write("\n")

        self.get_logger().info(f"Wrote datum to {self.config.output_file}")

    def compute_time_elapsed(self) -> float:
        if not self.samples:
            return 0.0

        seconds_diff = self.samples[-1].header.stamp.sec - self.samples[0].header.stamp.sec
        nanoseconds_diff = self.samples[-1].header.stamp.nanosec - self.samples[0].header.stamp.nanosec
        return seconds_diff + nanoseconds_diff / 1e9


def main() -> None:
    rclpy.init()
    node = GpsOriginCalculator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
