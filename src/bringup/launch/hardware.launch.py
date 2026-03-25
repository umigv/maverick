from bringup.launch_utils import MODES, Mode, bringup_share, format_mode_description, load_frames, load_gps_file
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from typing_extensions import assert_never


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode: Mode = LaunchConfiguration("mode").perform(context)
    course = LaunchConfiguration("course").perform(context)
    gps_file = load_gps_file(course)

    base_params = [
        f"{bringup_share()}/config/sensors/vectornav.yaml",
        {"imuFrameId": frames["imu_frame"]},
        {"insFrameId": frames["base_frame"]},
        {"gnssAFrameId": frames["gnss_a_frame"]},
        {"gnssBFrameId": frames["gnss_b_frame"]},
        {"mapFrameId": frames["map_frame"]},
    ]

    match mode:
        case "autonav":
            datum = gps_file["datum"]
            params = [*base_params, {"datum": [datum["latitude"], datum["longitude"], datum["altitude"]]}]
        case "self_drive":
            params = base_params
        case "nav_test":
            return []
        case _:
            assert_never(mode)

    return [
        Node(
            package="vectornav_driver",
            executable="vectornav_driver",
            name="vectornav_driver",
            output="screen",
            parameters=params,
            remappings=[
                ("vectornav/raw/imu", "imu/raw"),
                ("vectornav/raw/navsatfix", "gps/raw"),
                ("vectornav/raw/twist_with_covariance_stamped", "ins_vel/raw"),
                ("vectornav/raw/odometry", "odom/global"),
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description=format_mode_description(
                    {
                        "autonav": "Vectornav output + odom",
                        "self_drive": "Vectornav output",
                        "nav_test": "no sensors",
                    }
                ),
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load map and GPS datum from",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
