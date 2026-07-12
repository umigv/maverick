# occupancy_grid_simulator
Simulates a robot-centric occupancy grid from a static obstacle map for use in simulation, replacing a real perception stack. Loads a JSON map file describing the environment, then publishes a local occupancy grid around the robot at a fixed rate based on the robot's current pose.

## Map File Format
The map file is a JSON file with the following structure:
```json
{
  "resolution_m": 0.1,
  "obstacles": [[x0, y0], [x1, y1], ...],
  "lane_lines": [[x2, y2], [x3, y3], ...]
}
```
- `resolution_m` - Cell size in meters
- `obstacles` - List of `[x, y]` cell indices (integers) that are occupied by obstacles
- `lane_lines` - List of `[x, y]` cell indices (integers) that are occupied by lane lines

## Sensor Model
Each published grid is built in four steps, in order:
1. **World lookup** - Each cell of the robot-centric grid is transformed to world coordinates using the ground-truth pose and marked occupied if it lands on an obstacle cell.
2. **Occlusion** - Every cell whose line of sight from the robot passes through an obstacle is also marked occupied, mimicking a camera that cannot see behind obstacles. Note that occluded cells become occupied, not unknown.
3. **Lane lines** - Lane line cells are stamped on after occlusion, so they are always visible: they neither cast shadows nor get hidden behind obstacles.
4. **Blind spot** - A forward-pointing triangle of size `robot_blind_spot_height_m` of unknown cells is drawn in front of the robot, overwriting everything else. This simulates the camera's blind spot under the robot.

The blind spot is why [autonav_goal_selection](../../navigation/autonav_goal_selection/README.md) and [path_planning](../../navigation/path_planning/README.md) treat unknown cells within a forward / sideways region around the robot as drivable (`max_unknown_forward_m` / `max_unknown_sideways_m`) - without that carve-out, the robot would consider its own position non-traversable.

## Subscribed Topics
- `odom` (`nav_msgs/Odometry`) - Robot pose in the map frame

## Published Topics
- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Robot-centric occupancy grid stamped in `base_frame_id`, published periodically
- `occupancy_grid/ground_truth` (`nav_msgs/OccupancyGrid`) - Full static map (obstacles and lane lines) in `map` frame

`occupancy_grid/ground_truth` is latched: publisher and subscribers must both use `utils.qos.LATCHED` - see the [utils README](../../core/utils/README.md#utilsqos).

## Grid Conventions
Follows the standard [`nav_msgs/OccupancyGrid`](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html) conventions:
- Row major, +x forward, +y left of robot
- `info.origin` is the bottom-left corner of the grid in the robot frame
- Cell values: `0` = free, `100` = occupied, `-1` = unknown (only in the blind spot)

The robot-centric grid is sized `width_m × height_m` centered such that the robot sits at `(-offset_x_m, -offset_y_m)` within the grid (by default, horizontally centered and offset forward so the grid extends mostly in front of the robot).
