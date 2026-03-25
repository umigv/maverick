from bringup.launch_utils import MODES
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description="Operation mode, passed through to hardware/simulation and localization launch files",
            ),
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                choices=["true", "false"],
                description="Use simulation instead of hardware sensors",
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile (required for mode:=autonav or simulation:=true)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "core.launch.py"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "hardware.launch.py"])),
                condition=UnlessCondition(LaunchConfiguration("simulation")),
                launch_arguments=[
                    ("mode", LaunchConfiguration("mode")),
                    ("course", LaunchConfiguration("course")),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "simulation.launch.py"])),
                condition=IfCondition(LaunchConfiguration("simulation")),
                launch_arguments=[
                    ("course", LaunchConfiguration("course")),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "localization.launch.py"])
                ),
                launch_arguments=[
                    ("mode", LaunchConfiguration("mode")),
                    ("course", LaunchConfiguration("course")),
                ],
            ),
        ]
    )
