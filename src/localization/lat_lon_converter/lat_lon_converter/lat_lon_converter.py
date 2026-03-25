import rclpy
import utils.config
from geometry_msgs.msg import Point
from pyproj import Transformer
from rclpy.node import Node
from robot_localization.srv import FromLL

from .lat_lon_converter_config import LatLonConverterConfig


class LatLonConverter(Node):
    def __init__(self) -> None:
        super().__init__("lat_lon_converter")
        self.config = utils.config.load(self, LatLonConverterConfig)
        lat, lon, alt = self.config.datum
        self.to_enu = Transformer.from_pipeline(f"""
            +proj=pipeline
            +step +proj=cart +ellps=WGS84
            +step +proj=topocentric +lat_0={lat} +lon_0={lon} +h_0={alt} +ellps=WGS84
        """)
        self.create_service(FromLL, "fromLL", self.from_ll_callback)
        self.get_logger().info(f"Ready. Datum: lat={lat}, lon={lon}, alt={alt}")

    def from_ll_callback(self, request: FromLL.Request, response: FromLL.Response) -> FromLL.Response:
        ll = request.ll_point
        x, y, z = self.to_enu.transform(ll.longitude, ll.latitude, ll.altitude)
        response.map_point = Point(x=x, y=y, z=z)
        return response


def main() -> None:
    rclpy.init()
    node = LatLonConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
