from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cv_self_drive",
            executable="func_tests_occ_grid",
            name="self_drive_node",
            output="screen",
            parameters=[
                {"function_type": "right",
                  "hsv_json_key": "1"}
            ]
        )
    ])