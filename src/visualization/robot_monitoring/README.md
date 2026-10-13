# robot_monitoring

ROS 2 nodes for robot debugging and data visualization. Each tool has its own executable so it can be run independently.

## Path history

Displays the latest planned path and a trail of robot odometry positions as ROS point markers in Foxglove or RViz.

```bash
just build-package robot_monitoring --symlink-install
source install/setup.bash
ros2 run robot_monitoring path_history
```

Run the Foxglove bridge using `ros2 launch bringup visualization.launch.py`, connect to `ws://localhost:8765`, and enable both marker topics in the 3D panel. Use a display frame connected to the input frames by TF (normally `odom`).

### Subscribed topics

- `smoothed_path` (`nav_msgs/Path`): current planned path; replaces the previous plan.
- `odom/local` (`nav_msgs/Odometry`): estimated robot position; adds a trail point after at least 0.5 m of XY movement. Changing the odometry frame clears the trail.

### Published topics

- `planned_path_points` (`visualization_msgs/Marker`): planned path points.
- `actual_path_points` (`visualization_msgs/Marker`): recorded odometry trail.

Input topics can be remapped, for example to use simulation ground truth:

```bash
ros2 run robot_monitoring path_history --ros-args -r odom/local:=/odom/ground_truth
```

The trail is an odometry estimate unless a ground-truth source is used. History is stored in memory without a size limit and resets when the node restarts.

## Adding a tool

Add a module under `robot_monitoring/`, register its `main` function in `setup.py` under `console_scripts`, declare its ROS dependencies in `package.xml`, and document its topics and run command here.
