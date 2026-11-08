# robot_monitoring

This is the packagae that supports debugging features such as diagnose the robot's key nodes or save the path history in `Floxglove`. Please note that it is already launch the default settings when using `visualization.launch.py` launch file.

Run the Foxglove bridge `ros2 launch bringup visualization.launch.py`, connect to `ws://localhost:8765`, and enable both marker topics in the 3D panel. Use a display frame connected to the input frames by TF (normally `odom`).

## Path history

Displays the latest planned path of robot odometry positions as ROS point markers in `Foxglove`.

### Subscribed topics

- `smoothed_path` (`nav_msgs/Path`): current planned path; replaces the previous plan.
- `odom/local` (`nav_msgs/Odometry`): estimated robot position.

### Published topics

- `planned_path_points` (`visualization_msgs/Marker`): planned path points.
- `actual_path_points` (`visualization_msgs/Marker`): recorded odometry trail.

## Topic Diagnostic

Display the summary of important topics status `OK`, `WARN`, `ERROR` and its subsriber count. You can see it in `Foxglove`. You can click to `Add panel` in `Foxglove` then search for `Diagnostics-Detail (ROS)` or `Diagnostics-Summary (ROS)` to view the status. It sevres as the starting point when debugging.

### Subscribed topics

Each diagnostic node checks one topic, selected by its `topic` and `msg_type` parameters. The visualization launch file currently configures:

| Topic          | Configured message type          |
| -------------- | -------------------------------- |
| `/odom/local`  | `nav_msgs/msg/Odometry`          |
| `/cmd_vel`     | `geometry_msgs/msg/TwistStamped` |
| `/gps/raw`     | `sensor_msgs/msg/NavSatFix`      |
| `/imu/raw`     | `sensor_msgs/msg/Imu`            |
| `/nav_cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| `/path`        | `nav_msgs/msg/Odometry`          |

The configured type must match the publisher's actual type. Use `ros2 topic type /topic_name` to verify it.

### Published topics

- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): shared output from all topic checks, published once per second by each node. Each status is named `Debugging [<topic>]` and includes publisher count, publisher node names, subscriber count, and time since the last message. The subscriber count includes the diagnostic node itself.

Inspect the reports in the Foxglove diagnostics panels or in a terminal:

```bash
ros2 topic echo /diagnostics
```

### Add another topic check

1. Find the message type while the topic's publisher is running:

   ```bash
   ros2 topic type /battery_state
   ```

2. Add a `Node` to the `LaunchDescription` list in [visualization.launch.py](../../bringup/launch/visualization.launch.py). Use a unique node name and the type returned above. For example, for a topic publishing `sensor_msgs/msg/BatteryState`:

   ```python
   Node(
       package="robot_monitoring",
       executable="topic_diagnostic",
       name="battery_diagnostic",
       parameters=[
           {"topic": "/battery_state"},
           {"msg_type": "sensor_msgs/msg/BatteryState"},
       ],
   ),
   ```

3. Build from the workspace root and restart the visualization launch:

   ```bash
   just build-package bringup --symlink-install
   source install/setup.bash
   ros2 launch bringup visualization.launch.py
   ```

For a temporary check without editing the launch file, run a separate node:

```bash
ros2 run robot_monitoring topic_diagnostic --ros-args \
  -r __node:=battery_diagnostic \
  -p topic:=/battery_state \
  -p msg_type:=sensor_msgs/msg/BatteryState
```
