from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                parameters=[
                    {"port": 8765},
                    {"address": "0.0.0.0"},
                    {"tls": False},
                    {"topic_whitelist": [".*"]},
                    {"param_whitelist": [".*"]},
                    {"service_whitelist": [".*"]},
                    {"send_buffer_limit": 10000000},
                    {"use_sim_time": False},
                    {"num_threads": 4},
                ],
            ),
            Node(
                package="occupancy_grid_visualization",
                executable="foxglove_voxel",
                name="occupancy_grid_voxel_visualization",
            ),
        ]
    )
