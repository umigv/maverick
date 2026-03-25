from datetime import datetime, timezone

import rclpy
import utils.config
from builtin_interfaces.msg import Time
from pyubx2 import UBX_PROTOCOL, UBXMessage, UBXReader
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import Serial
from std_msgs.msg import Header

from .ublox_driver_config import UbloxDriverConfig

UBX_FIX_TYPE_NO_FIX = 0
UBX_FIX_TYPE_TIME_ONLY = 5


class UbloxDriver(Node):
    def __init__(self) -> None:
        super().__init__("ublox_driver")

        self.config: UbloxDriverConfig = utils.config.load(self, UbloxDriverConfig)

        self.publisher = self.create_publisher(NavSatFix, "ublox/gps", 10)

        self.stream = Serial(self.config.serial_port, 460800, timeout=0.1)
        self.ubx_reader = UBXReader(self.stream, protfilter=UBX_PROTOCOL)

        self.create_timer(self.config.poll_period_s, self.poll)

    def poll(self) -> None:
        try:
            _, msg = self.ubx_reader.read()
        except Exception as e:
            self.get_logger().error(f"Error reading GPS msg: {e}")
            return

        if msg is None:
            return

        if msg.identity not in ("NAV-PVT", "NAV2-PVT"):
            return

        if msg.fixType in (UBX_FIX_TYPE_NO_FIX, UBX_FIX_TYPE_TIME_ONLY):
            self.get_logger().debug(f"Dropping GPS msg with no fix: {msg}")
            return

        if not msg.gnssFixOk:
            self.get_logger().debug(f"Dropping GPS msg with fix not ok: {msg}")
            return

        if msg.invalidLlh:
            self.get_logger().debug(f"Dropping GPS msg with invalid LLH: {msg}")
            return

        self.get_logger().debug(f"Publishing GPS Msg: {msg}")
        self.publish_from_pvt(msg)

    def publish_from_pvt(self, data: UBXMessage) -> None:
        hcov = max(data.hAcc * 1e-3, 0.05) ** 2
        vcov = max(data.vAcc * 1e-3, 0.08) ** 2
        self.publisher.publish(
            NavSatFix(
                header=Header(
                    stamp=self.resolve_timestamp(data),
                    frame_id=self.config.ublox_frame_id,
                ),
                status=NavSatStatus(
                    status=NavSatStatus.STATUS_GBAS_FIX if data.diffSoln else NavSatStatus.STATUS_FIX,
                    service=NavSatStatus.SERVICE_GPS,
                ),
                latitude=data.lat,
                longitude=data.lon,
                altitude=data.hMSL * 1e-3,
                position_covariance=[hcov, 0.0, 0.0, 0.0, hcov, 0.0, 0.0, 0.0, vcov],
                position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            )
        )

    def resolve_timestamp(self, data: UBXMessage) -> Time:
        if not (data.validDate and data.validTime and data.fullyResolved):
            return self.get_clock().now().to_msg()

        seconds = datetime(
            year=data.year,
            month=data.month,
            day=data.day,
            hour=data.hour,
            minute=data.min,
            second=data.second,
            tzinfo=timezone.utc,
        ).timestamp()

        if data.nano < 0:
            return Time(sec=int(seconds) - 1, nanosec=int(data.nano) + 1_000_000_000)

        return Time(sec=int(seconds), nanosec=int(data.nano))


def main() -> None:
    rclpy.init()
    node = UbloxDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
