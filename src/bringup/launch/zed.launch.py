from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="zed_publisher",
                executable="zed_publisher",
                name="zed_publisher",
                output="screen",
                parameters=[
                    {
                        "fps": 30,
                        "brightness": 4,
                        "contrast": 4,
                        "hue": 0,
                        "saturation": 4,
                        "sharpness": 4,
                        "gamma": 5,
                        "gain": 50,
                        "exposure": 50,
                        "white_balance_temperature": 4600,
                    }
                ],
            ),
        ]
    )


# resolution: sl.RESOLUTION = sl.RESOLUTION.HD720
#     fps: int = 30
#     depth_mode: sl.DEPTH_MODE = sl.DEPTH_MODE.NEURAL

#     brightness: int = 4  # 0-8
#     contrast: int = 4  # 0-8
#     hue: int = 0  # 0-11
#     saturation: int = 4  # 0-8
#     sharpness: int = 4  # 0-8
#     gamma: int = 5  # 1-9

#     gain: int = 50  # 0-100
#     exposure: int = 50  # 0-100

#     white_balance_temperature: int = 4600  # 2800-6500

#     auto_exposure: bool = True
#     auto_white_balance: bool = True
