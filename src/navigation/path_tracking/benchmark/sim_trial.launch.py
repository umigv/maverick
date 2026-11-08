"""Simulation trial for comparing path tracking controllers without editing the production launch files.

Brings up base.launch.py (simulation:=true) and navigation.launch.py unchanged, with parameter overrides scoped to each
include. Each overridden name is declared by exactly one node (path_tracking, occupancy_grid_simulator or
occupancy_grid_transform); other nodes ignore them.

The grid width and inflation overrides are empty by default, which keeps the production values.

Usage:
    pixi run ros2 launch src/navigation/path_tracking/benchmark/sim_trial.launch.py course:=test2 algorithm:=mppi
    pixi run ros2 launch src/navigation/path_tracking/benchmark/sim_trial.launch.py algorithm:=differential_drive \
        target_speed_mps:=2.0
"""

from launch import Action, LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    mode = LaunchConfiguration("mode").perform(context)
    course = LaunchConfiguration("course").perform(context)
    algorithm = LaunchConfiguration("algorithm").perform(context)
    target_speed = LaunchConfiguration("target_speed_mps").perform(context)
    grid_height = LaunchConfiguration("grid_height_m").perform(context)
    inflation = LaunchConfiguration("inflation_radius_cells").perform(context)
    launch_dir = FindPackageShare("bringup").perform(context) + "/launch"

    simulation_overrides: list[Action] = []
    if grid_height:
        simulation_overrides.append(SetParameter(name="height_m", value=float(grid_height)))
        simulation_overrides.append(SetParameter(name="offset_y_m", value=-float(grid_height) / 2))

    overrides: list[Action] = [SetParameter(name="algorithm", value=algorithm)]
    if target_speed:
        overrides.append(SetParameter(name=f"{algorithm}.target_speed_mps", value=float(target_speed)))
    if inflation:
        overrides.append(SetParameter(name="inflation_params.inflation_radius_cells", value=int(inflation)))

    return [
        GroupAction(
            [
                *simulation_overrides,
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(f"{launch_dir}/base.launch.py"),
                    launch_arguments=[("mode", mode), ("simulation", "true"), ("course", course)],
                ),
            ],
            scoped=True,
        ),
        GroupAction(
            [
                *overrides,
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(f"{launch_dir}/navigation.launch.py"),
                    launch_arguments=[("mode", mode), ("course", course)],
                ),
            ],
            scoped=True,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="nav_test"),
            DeclareLaunchArgument("course", default_value="test2"),
            DeclareLaunchArgument("algorithm", default_value="differential_drive"),
            DeclareLaunchArgument(
                "target_speed_mps", default_value="", description="Override <algorithm>.target_speed_mps if set"
            ),
            DeclareLaunchArgument("grid_height_m", default_value="", description="occupancy_grid_simulator width"),
            DeclareLaunchArgument(
                "inflation_radius_cells", default_value="", description="occupancy_grid_transform inflation core"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
