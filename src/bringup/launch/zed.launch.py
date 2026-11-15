from typing import Any

from bringup.launch_utils import bringup_share, load_frames
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare

NAMESPACE = "zed"
CONTAINER_NAME = "zed_container"
CAMERA_MODEL = "zed2i"


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    cameras = {
        frames["zed_left_frame"].removesuffix("_camera_link"): LaunchConfiguration("left_serial").perform(context),
        frames["zed_right_frame"].removesuffix("_camera_link"): LaunchConfiguration("right_serial").perform(context),
    }

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace=NAMESPACE,
        package="rclcpp_components",
        executable="component_container_isolated",
        arguments=["--use_multi_threaded_executor"],
        output="screen",
        composable_node_descriptions=[],
    )

    zed_launch = PathJoinSubstitution([FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"])
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments=[
                ("camera_model", CAMERA_MODEL),
                ("camera_name", name),
                ("serial_number", serial),
                ("namespace", NAMESPACE),
                ("container_name", CONTAINER_NAME),
                ("ros_params_override_path", f"{bringup_share()}/config/hardware/zed.yaml"),
                ("publish_urdf", "true"),
                ("publish_tf", "false"),
                ("publish_map_tf", "false"),
                ("publish_imu_tf", "false"),
            ],
        )
        for name, serial in cameras.items()
    ]

    return [container, *includes]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # TODO: fill in with real serial numbers
            DeclareLaunchArgument("left_serial", default_value="0"),
            DeclareLaunchArgument("right_serial", default_value="0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
