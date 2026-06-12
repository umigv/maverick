from bringup.launch_utils import MODES, Mode, bringup_share, load_frames, load_gps_file
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

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[
            f"{bringup_share()}/config/localization/ekf.yaml",
            {"map_frame": frames["map_frame"]},
            {"odom_frame": frames["odom_frame"]},
            {"base_link_frame": frames["base_frame"]},
            {"world_frame": frames["odom_frame"]},
        ],
        remappings=[
            ("odometry/filtered", "odom/local"),
        ],
    )

    enc_odom_node = Node(
        package="enc_odom_publisher",
        executable="enc_odom_publisher",
        name="enc_odom_publisher",
        output="screen",
        parameters=[
            {"odom_frame_id": frames["odom_frame"]},
            {"base_frame_id": frames["base_frame"]},
        ],
        remappings=[
            ("enc_vel", "enc_vel/raw"),
            ("odom", "odom/local"),
        ],
    )

    identity_map_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_publisher",
        output="screen",
        arguments=["--frame-id", frames["map_frame"], "--child-frame-id", frames["odom_frame"]],
    )

    map_odom_publisher_node = Node(
        package="map_odom_publisher",
        executable="map_odom_publisher",
        name="map_odom_publisher",
        output="screen",
        parameters=[
            {"map_frame_id": frames["map_frame"]},
            {"odom_frame_id": frames["odom_frame"]},
            {"base_frame_id": frames["base_frame"]},
        ],
        remappings=[
            ("odom", "odom/global"),
        ],
    )

    lat_lon_converter_node = Node(
        package="lat_lon_converter",
        executable="lat_lon_converter",
        name="lat_lon_converter",
        output="screen",
        parameters=[
            {"datum": [gps_file["datum"]["latitude"], gps_file["datum"]["longitude"], gps_file["datum"]["altitude"]]},
        ],
    )

    match mode:
        case "autonav":
            return [ekf_local_node, map_odom_publisher_node, lat_lon_converter_node]
        case "self_drive":
            return [ekf_local_node, identity_map_odom_node]
        case "nav_test":
            return [enc_odom_node, identity_map_odom_node]
        case _:
            assert_never(mode)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description="See bringup/README.md",
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="See bringup/README.md",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
