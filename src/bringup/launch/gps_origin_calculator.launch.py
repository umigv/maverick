from bringup.launch_utils import bringup_share, get_package_share_directory, load_frames
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    course = LaunchConfiguration("course").perform(context)
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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gps_driver_node = Node(
        package="vectornav_driver",
        executable="vectornav_driver",
        name="vectornav_driver",
        output="screen",
        parameters=[
            f"{bringup_share()}/config/hardware/vectornav.yaml",
            {"imuFrameId": frames["imu_frame"]},
            {"insFrameId": frames["base_frame"]},
            {"gnssAFrameId": frames["gnss_a_frame"]},
            {"gnssBFrameId": frames["gnss_b_frame"]},
            {"mapFrameId": frames["map_frame"]},
        ],
        remappings=[
            ("vectornav/raw/navsatfix", "gps/raw"),
        ],
    )

    gps_origin_node = Node(
        package="gps_origin_calculator",
        executable="gps_origin_calculator",
        output="screen",
        parameters=[{"output_file": f"{bringup_share()}/courses/{course}/gps.json"}],
        remappings=[("gps", "gps/raw")],
    )

    shutdown_on_completion = RegisterEventHandler(
        OnProcessExit(
            target_action=gps_origin_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return [robot_state_publisher_node, gps_driver_node, gps_origin_node, shutdown_on_completion]


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
