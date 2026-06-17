# occupancy_grid_simulator
Simulates a robot-centric occupancy grid from a static obstacle map for use in simulation, replacing
a real perception stack. Loads a JSON map file describing the environment, then publishes a
local occupancy grid around the robot at a fixed rate based on the robot's current pose.

## Map File Format
The map file is a JSON file with the following structure:
```json
{
  "resolution_m": 0.1,
  "obstacles": [[x0, y0], [x1, y1], ...]
}
```
- `resolution_m` - cell size in meters
- `obstacles` - list of `[x, y]` cell indices (integers) that are occupied

## Subscribed Topics
- `odom` (`nav_msgs/Odometry`) - Robot pose in the map frame

## Published Topics
- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Robot-centric occupancy grid stamped in `base_frame_id`, published 
periodically
- `occupancy_grid/ground_truth` (`nav_msgs/OccupancyGrid`) - Full static obstacle map in `map` frame

`occupancy_grid/ground_truth` is latched: publisher and subscribers must both use `utils.qos.LATCHED` — see the
[utils README](../../core/utils/README.md#utilsqos).

## Grid Conventions
Follows the standard [`nav_msgs/OccupancyGrid`](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) 
conventions:
- Row major, +x forward, +y left of robot
- `info.origin` is the bottom-left corner of the grid in the robot frame
- Cell values: `0` = free, `100` = occupied

The robot-centric grid is sized `width_m × height_m` centered such that the robot sits at
`(-offset_x_m, -offset_y_m)` within the grid (by default, horizontally centered and offset
forward so the grid extends mostly in front of the robot).
