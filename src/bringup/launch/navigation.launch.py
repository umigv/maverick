from bringup.launch_utils import MODES, Mode, bringup_share, format_mode_description, load_frames
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
            ("transformed_occupancy_grid", "occupancy_grid/transformed"),
            ("inflated_occupancy_grid", "occupancy_grid/inflated"),
        ],
    )

    autonav_mission_control_node = Node(
        package="autonav_mission_control",
        executable="autonav_mission_control",
        name="autonav_mission_control",
        parameters=[
            {"waypoints_file_path": f"{bringup_share()}/courses/{course}/gps.json"},
            {"map_frame_id": frames["map_frame"]},
        ],
        remappings=[
            ("odom", "odom/global"),
            ("fromLL", "fromLL"),
            ("mission_state", "mission_state"),
            ("request_recovery", "request_recovery"),
            ("recovery_complete", "recovery_complete"),
        ],
    )

    autonav_goal_selection_node = Node(
        package="autonav_goal_selection",
        executable="autonav_goal_selection",
        name="autonav_goal_selection",
        parameters=[
            {"world_frame_id": frames["odom_frame"]},
        ],
        remappings=[
            ("occupancy_grid", "occupancy_grid/transformed"),
            ("odom", "odom/local"),
            ("mission_state", "mission_state"),
            ("request_recovery", "request_recovery"),
            ("goal", "goal"),
            ("goal_selection_debug", "goal_selection_debug"),
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
            ("occupancy_grid", "occupancy_grid/inflated"),
            ("odom", "odom/local"),
            ("goal", "goal"),
            ("path", "path"),
        ],
    )

    path_smoothing_node = Node(
        package="path_smoothing",
        executable="path_smoothing",
        name="path_smoothing",
        remappings=[
            ("path", "path"),
            ("smoothed_path", "smoothed_path"),
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
            ("path", "smoothed_path"),
            ("nav_cmd_vel", "nav_cmd_vel"),
            ("smoothed_path", "smoothed_path"),
        ],
    )

    recovery_behavior_node = Node(
        package="recovery_behavior",
        executable="recovery_behavior",
        name="recovery_behavior",
        output="screen",
    )

    shutdown_on_completion = RegisterEventHandler(
        OnProcessExit(
            target_action=autonav_mission_control_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    match mode:
        case "autonav":
            return [
                occupancy_grid_transform_node,
                path_planning_node,
                path_smoothing_node,
                path_tracking_node,
                autonav_mission_control_node,
                autonav_goal_selection_node,
                recovery_behavior_node,
                shutdown_on_completion,
            ]
        case "self_drive" | "nav_test":
            return [
                occupancy_grid_transform_node,
                path_planning_node,
                path_smoothing_node,
                path_tracking_node,
                recovery_behavior_node,
            ]
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
                        "autonav": "occupancy grid transform + path planning + path smoothing + path tracking + mission control + autonav goal selection + recovery",
                        "self_drive": "occupancy grid transform + path planning + path smoothing + path tracking + recovery",
                        "nav_test": "occupancy grid transform + path planning + path smoothing + path tracking + recovery",
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
