from bringup.launch_utils import bringup_share, load_frames, load_gps_file
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    course = LaunchConfiguration("course").perform(context)
    share = bringup_share()
    gps_file = load_gps_file(course)

    sensor_simulator_node = Node(
        package="sensor_simulator",
        executable="sensor_simulator",
        name="sensor_simulator",
        output="screen",
        parameters=[
            f"{share}/config/sensors/sensor_simulator.yaml",
            {"map_frame_id": frames["map_frame"]},
            {"base_frame_id": frames["base_frame"]},
            {"ground_truth_base_frame_id": frames["ground_truth_base_frame"]},
            {"vn300.imu_frame_id": frames["imu_frame"]},
            {"vn300.ins_frame_id": frames["base_frame"]},
            {"datum": [gps_file["datum"]["latitude"], gps_file["datum"]["longitude"], gps_file["datum"]["altitude"]]},
        ],
        remappings=[
            ("gps", "gps/raw"),
            ("enc_vel", "enc_vel/raw"),
            ("imu", "imu/raw"),
            ("ins_vel", "ins_vel/raw"),
            ("odom", "odom/global"),
            ("odom/ground_truth", "odom/ground_truth"),
        ],
    )

    occupancy_grid_simulator_node = Node(
        package="occupancy_grid_simulator",
        executable="occupancy_grid_simulator",
        name="occupancy_grid_simulator",
        output="screen",
        parameters=[
            {"map_file_path": f"{share}/courses/{course}/map.json"},
            {"map_frame_id": frames["map_frame"]},
            {"base_frame_id": frames["base_frame"]},
            {"ground_truth_base_frame_id": frames["ground_truth_base_frame"]},
        ],
        remappings=[
            ("odom", "odom/ground_truth"),
            ("occupancy_grid", "occupancy_grid/raw"),
            ("occupancy_grid/ground_truth", "occupancy_grid/ground_truth"),
        ],
    )

    return [sensor_simulator_node, occupancy_grid_simulator_node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load map and GPS datum from",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
