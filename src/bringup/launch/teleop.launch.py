from bringup.launch_utils import bringup_share, load_frames
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = ("ps4", "xbox")


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    teleop_params = f"{bringup_share()}/config/teleop/teleop_{controller}.yaml"

    return [
        Node(
            package="joy",
            executable="joy_node",
            name="joy",
            output="screen",
            parameters=[
                teleop_params,
            ],
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy",
            output="screen",
            parameters=[
                teleop_params,
                {"frame": frames["base_frame"]},
            ],
            remappings=[
                ("cmd_vel", "teleop_cmd_vel"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                choices=list(CONTROLLERS),
                description="Controller profile",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
