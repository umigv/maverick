import cv2
import pyzed.sl as sl
import rclpy
import utils.config
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from .zed_publisher_config import ZedPublisherConfig


class ZedPublisher(Node):
    def __init__(self) -> None:
        super().__init__("zed-publisher-node")

        self.config: ZedPublisherConfig = utils.config.load(self, ZedPublisherConfig)

        left_serial = 123
        right_serial = 456

        self.left_cam = self._initialize_camera(left_serial, "left")
        self.right_cam = self._initialize_camera(right_serial, "right")

        self.left_rgb_pub = self.create_publisher(Image, "/zed/left/rgb/image", 1)
        self.left_depth_pub = self.create_publisher(Image, "/zed/left/depth/image", 1)

        self.right_rgb_pub = self.create_publisher(Image, "/zed/right/rgb/image", 1)
        self.right_depth_pub = self.create_publisher(Image, "/zed/right/depth/image", 1)

        self.bridge = CvBridge()

        self.left_image = sl.Mat()
        self.left_depth = sl.Mat()

        self.right_image = sl.Mat()
        self.right_depth = sl.Mat()

    def _initialize_camera(self, serial: int, name: str) -> sl.Camera:
        params = sl.InitParameters()
        params.set_from_serial_number(serial)

        params.camera_resolution = self.config.resolution
        params.camera_fps = self.config.fps

        params.depth_mode = self.config.depth_mode

        camera = sl.Camera()

        status = camera.open(params)

        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"[zed-publisher-node] Failed to open {name} ZED (serial={serial}): {status}")

        camera.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, self.config.brightness)
        camera.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, self.config.contrast)
        camera.set_camera_settings(sl.VIDEO_SETTINGS.HUE, self.config.hue)
        camera.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, self.config.saturation)
        camera.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, self.config.sharpness)
        camera.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, self.config.gamma)

        camera.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, self.config.gain)
        camera.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, self.config.exposure)

        camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, self.config.white_balance_temperature)

        camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, int(self.config.auto_exposure))
        camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, int(self.config.auto_white_balance))

        self.get_logger().info(f"[zed-publisher-node] Opened {name} ZED (serial={serial})")

        return camera

    def _publish_frames(self) -> None:
        if self.left_cam.grab() == sl.ERROR_CODE.SUCCESS:
            self.left_cam.retrieve_image(self.left_image, sl.VIEW.LEFT)
            self.left_cam.retrieve_measure(self.left_depth, sl.MEASURE.DEPTH)

            left_bgra = self.left_image.get_data()
            left_rgb = cv2.cvtColor(left_bgra, cv2.COLOR_BGRA2RGB)

            left_rgb_msg = self.bridge.cv2_to_imgmsg(left_rgb, encoding="rgb8")

            left_depth = self.left_depth.get_data()
            left_depth_msg = self.bridge.cv2_to_imgmsg(left_depth, encoding="32FC1")

            stamp = self.get_clock().now().to_msg()
            left_rgb_msg.header.stamp = stamp
            left_depth_msg.header.stamp = stamp

            left_rgb_msg.header.frame_id = "zed_left_camera"
            left_depth_msg.header.frame_id = "zed_left_camera"

            self.left_rgb_pub.publish(left_rgb_msg)
            self.left_depth_pub.publish(left_depth_msg)

        if self.right_cam.grab() == sl.ERROR_CODE.SUCCESS:
            self.right_cam.retrieve_image(self.right_image, sl.VIEW.LEFT)
            self.right_cam.retrieve_measure(self.right_depth, sl.MEASURE.DEPTH)

            right_bgra = self.right_image.get_data()
            right_rgb = cv2.cvtColor(right_bgra, cv2.COLOR_BGRA2RGB)

            right_rgb_msg = self.bridge.cv2_to_imgmsg(right_rgb, encoding="rgb8")

            right_depth = self.right_depth.get_data()
            right_depth_msg = self.bridge.cv2_to_imgmsg(right_depth, encoding="32FC1")

            stamp = self.get_clock().now().to_msg()
            right_rgb_msg.header.stamp = stamp
            right_depth_msg.header.stamp = stamp

            right_rgb_msg.header.frame_id = "zed_right_camera"
            right_depth_msg.header.frame_id = "zed_right_camera"

            self.right_rgb_pub.publish(right_rgb_msg)
            self.right_depth_pub.publish(right_depth_msg)

    def run(self) -> None:
        self.create_timer(1.0 / self.config.fps, self._publish_frames)
        rclpy.spin(self)

    def destroy_node(self) -> None:
        self.left_cam.close()
        self.right_cam.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = ZedPublisher()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
