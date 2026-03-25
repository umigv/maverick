from bringup.launch_utils import MODES, Mode, bringup_share, format_mode_description, load_frames
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from typing_extensions import assert_never


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode: Mode = LaunchConfiguration("mode").perform(context)
    course = LaunchConfiguration("course").perform(context)

    occupancy_grid_transform_node = Node(
        package="occupancy_grid_transform",
        executable="occupancy_grid_transform",
        name="occupancy_grid_transform",
        parameters=[
            {"frame_id": frames["odom_frame"]},
        ],
        remappings=[
            ("occupancy_grid", "occupancy_grid/raw"),
            ("transformed_occupancy_grid", "occupancy_grid"),
        ],
    )

    path_tracking_node = Node(
        package="path_tracking",
        executable="path_tracking",
        name="path_tracking",
        parameters=[
            {"base_frame_id": frames["base_frame"]},
            {"odom_frame_id": frames["odom_frame"]},
        ],
        remappings=[
            ("odom", "odom/local"),
            ("path", "path"),
            ("nav_cmd_vel", "nav_cmd_vel"),
            ("smoothed_path", "smoothed_path"),
        ],
    )

    autonav_goal_selection_node = Node(
        package="autonav_goal_selection",
        executable="autonav_goal_selection",
        name="autonav_goal_selection",
        parameters=[
            {"waypoints_file_path": f"{bringup_share()}/courses/{course}/gps.json"},
            {"map_frame_id": frames["map_frame"]},
            {"world_frame_id": frames["odom_frame"]},
        ],
        remappings=[
            ("occupancy_grid", "occupancy_grid"),
            ("odom", "odom/local"),
            ("fromLL", "fromLL"),
            ("goal", "goal"),
            ("gps_waypoint", "gps_waypoint"),
        ],
    )

    path_planning_node = Node(
        package="path_planning",
        executable="path_planning",
        name="path_planning",
        parameters=[
            {"frame_id": frames["odom_frame"]},
        ],
        remappings=[
            ("occupancy_grid", "occupancy_grid"),
            ("odom", "odom/local"),
            ("goal", "goal"),
            ("path", "path"),
        ],
    )

    match mode:
        case "autonav":
            return [occupancy_grid_transform_node, path_tracking_node, path_planning_node, autonav_goal_selection_node]
        case "self_drive" | "nav_test":
            return [occupancy_grid_transform_node, path_tracking_node, path_planning_node]
        case _:
            assert_never(mode)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description=format_mode_description(
                    {
                        "autonav": "occupancy grid transform + path planning + path tracking + autonav goal selection",
                        "self_drive": "occupancy grid transform + path planning + path tracking",
                        "nav_test": "occupancy grid transform + path planning + path tracking",
                    }
                ),
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load waypoints from (required for autonav, autonav_sim)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
