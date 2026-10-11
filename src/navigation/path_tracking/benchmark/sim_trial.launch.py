"""Simulation trial for comparing path tracking controllers without editing the production launch files.

Brings up base.launch.py (simulation:=true) and navigation.launch.py unchanged, with parameter overrides scoped to each
include. Each overridden name is declared by exactly one node (path_tracking, occupancy_grid_simulator or
occupancy_grid_transform); other nodes ignore them.

The perception overrides adapt the scaled-down simulation courses so the planner can see a collision-free route on test3
(goal 7.5, 1.5): a smaller blind spot (default 1.25 m hides nearby obstacles, which the planner treats as traversable),
inflation reduced from 12 to 8 cells (0.40 m, just above the ~0.38 m robot half width), and a 9 m wide grid (default 5 m)
so the gap around the middle obstacle is in view. Pass an empty value to keep a node's default.

Usage:
    pixi run ros2 launch src/navigation/path_tracking/benchmark/sim_trial.launch.py course:=test3 algorithm:=mppi
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
    blind_spot = LaunchConfiguration("blind_spot_m").perform(context)
    grid_height = LaunchConfiguration("grid_height_m").perform(context)
    inflation = LaunchConfiguration("inflation_radius_cells").perform(context)
    launch_dir = FindPackageShare("bringup").perform(context) + "/launch"

    simulation_overrides: list[Action] = []
    if blind_spot:
        simulation_overrides.append(SetParameter(name="robot_blind_spot_height_m", value=float(blind_spot)))
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
            DeclareLaunchArgument("course", default_value="test3"),
            DeclareLaunchArgument("algorithm", default_value="differential_drive"),
            DeclareLaunchArgument(
                "target_speed_mps", default_value="", description="Override <algorithm>.target_speed_mps if set"
            ),
            DeclareLaunchArgument(
                "blind_spot_m", default_value="0.5", description="occupancy_grid_simulator blind spot"
            ),
            DeclareLaunchArgument("grid_height_m", default_value="9.0", description="occupancy_grid_simulator width"),
            DeclareLaunchArgument(
                "inflation_radius_cells", default_value="8", description="occupancy_grid_transform inflation core"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
