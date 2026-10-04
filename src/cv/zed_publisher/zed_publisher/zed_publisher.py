import pyzed.sl as sl
import utils.config
from rclpy.node import Node
from sensor_msgs.msg import Image

from .zed_publisher_config import ZedPublisherConfig


class ZedPublisher(Node):
    def __init__(self):
        self.node_name = "zed-publisher-node"
        super().__init__(self.node_name)

        self.config: ZedPublisherConfig = utils.config.load(self, ZedPublisherConfig)

        left_serial = 123
        right_serial = 456

        self.left_cam = self._initialize_camera(left_serial, "left")
        self.right_cam = self._initialize_camera(right_serial, "right")

        self.left_rgb_pub = self.create_publisher(Image, "/zed/left/rgb/image", 1)
        self.left_rgbd_pub = self.create_publisher(Image, "/zed/left/rgbd/image", 1)

        self.right_rgb_pub = self.create_publisher(Image, "/zed/right/rgb/image", 1)
        self.right_rgbd_pub = self.create_publisher(Image, "/zed/right/rgbd/image", 1)

    def _initialize_camera(self, serial: int, name: str) -> sl.Camera:
        params = sl.InitParameters()
        params.set_from_serial_number(serial)

        params.camera_resolution = self.config.resolution
        params.camera_fps = self.config.fps

        params.depth_mode = self.config.depth_mode

        camera = sl.Camera()

        status = camera.open(params)

        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"[{self.node_name}] Failed to open {name} ZED (serial={serial}): {status}")

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

        self.get_logger().info(f"[{self.node_name}] Opened {name} ZED (serial={serial})")

        return camera
