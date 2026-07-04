from typing import Any

from bringup.launch_utils import ESTOP_FILE_PATH, MODES, Mode, bringup_share, load_frames, load_gps_file
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from typing_extensions import assert_never


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode: Mode = LaunchConfiguration("mode").perform(context)
    course = LaunchConfiguration("course").perform(context)
    gps_file = load_gps_file(course)

    base_params = [
        f"{bringup_share()}/config/hardware/vectornav.yaml",
        {"imu_frame_id": frames["imu_frame"]},
        {"ins_frame_id": frames["base_frame"]},
        {"gnss_a_frame_id": frames["gnss_a_frame"]},
        {"gnss_b_frame_id": frames["gnss_b_frame"]},
        {"map_frame_id": frames["map_frame"]},
    ]

    vectornav_params: list | None = None
    match mode:
        case "autonav":
            datum = gps_file["datum"]
            vectornav_params = [*base_params, {"datum": [datum["latitude"], datum["longitude"], datum["altitude"]]}]
        case "self_drive":
            vectornav_params = [*base_params, {"require_attitude": False}]
        case "nav_test":
            pass
        case _:
            assert_never(mode)

    return [
        Node(
            package="estop_driver",
            executable="estop_driver",
            name="estop_driver",
            output="screen",
            parameters=[{"estop_file_path": ESTOP_FILE_PATH}],
        ),
        Node(
            package="led_driver",
            executable="led_driver",
            name="led_driver",
            output="screen",
            parameters=[{"estop_file_path": ESTOP_FILE_PATH}],
        ),
        Node(
            package="odrive_driver",
            executable="odrive_driver",
            name="odrive_driver",
            output="screen",
            parameters=[
                {"frame_id": frames["base_frame"]},
                {"estop_file_path": ESTOP_FILE_PATH},
            ],
            remappings=[
                ("enc_vel", "enc_vel/raw"),
            ],
        ),
        *(
            [
                Node(
                    package="vectornav_driver",
                    executable="vectornav_driver",
                    name="vectornav_driver",
                    output="screen",
                    parameters=vectornav_params,
                    remappings=[
                        ("vectornav/imu", "imu/raw"),
                        ("vectornav/fix", "gps/raw"),
                        ("vectornav/velocity", "ins_vel/raw"),
                        ("vectornav/odom", "odom/global"),
                    ],
                ),
            ]
            if vectornav_params is not None
            else []
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", choices=MODES),
            DeclareLaunchArgument("course", default_value="default"),
            OpaqueFunction(function=launch_setup),
        ]
    )
