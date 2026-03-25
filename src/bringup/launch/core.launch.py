from ament_index_python.packages import get_package_share_directory
from bringup.launch_utils import bringup_share, load_frames
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    frames = load_frames()
    twist_mux_params = f"{bringup_share()}/config/core/twist_mux.yaml"
    urdf = f"{get_package_share_directory('maverick_description')}/urdf/maverick.xacro"

    # fmt: off
    robot_description = ParameterValue(
        Command(
            [
                "xacro ", urdf,
                " base_frame_id:=", frames["base_frame"],
                " imu_name:=", frames["imu_frame"].removesuffix("_link"),
                " gnss_a_name:=", frames["gnss_a_frame"].removesuffix("_link"),
                " gnss_b_name:=", frames["gnss_b_frame"].removesuffix("_link"),
            ]
        ), value_type=str,
    )
    # fmt: on

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="state_machine",
                executable="state_machine",
                name="state_machine",
                output="screen",
                remappings=[
                    ("state", "state"),
                    ("state/set_no_mans_land", "state/set_no_mans_land"),
                    ("state/set_recovery", "state/set_recovery"),
                ],
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[
                    twist_mux_params,
                ],
                remappings=[
                    ("cmd_vel_out", "cmd_vel"),
                ],
            ),
        ]
    )
