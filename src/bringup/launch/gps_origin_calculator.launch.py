from typing import Any

from bringup.launch_utils import gps_file_path
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[LaunchDescriptionEntity]:
    course = LaunchConfiguration("course").perform(context)

    gps_origin_node = Node(
        package="gps_origin_calculator",
        executable="gps_origin_calculator",
        output="screen",
        parameters=[{"output_file": gps_file_path(course)}],
        remappings=[("gps", "gps/raw")],
    )

    shutdown_on_completion = RegisterEventHandler(
        OnProcessExit(
            target_action=gps_origin_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return [gps_origin_node, shutdown_on_completion]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to write the computed GPS datum into.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
