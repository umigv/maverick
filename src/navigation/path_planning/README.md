# path_planning
Path planning from the robot's position to a local goal using A* and string-pulling.

On each received goal, runs A* through the occupancy grid to find a path to the goal. Unknown cells within a  forward /
sideways region around the robot are treated as traversable, allowing planning directly from the robot's position even 
when cells beneath it are unobserved. If the goal is unreachable, the path leads to the closest reachable cell instead. 
The raw A* path is then simplified with string-pulling, which removes unnecessary waypoints by replacing sequences of 
collinear-or-visible points with direct segments.

## Subscribed Topics
- `odom` (`nav_msgs/msg/Odometry`) — Robot pose in the planning frame
- `occupancy_grid` (`nav_msgs/msg/OccupancyGrid`) — Occupancy grid used for path planning
- `goal` (`geometry_msgs/msg/PointStamped`) — Local goal to plan towards; triggers replanning on each message.
Transformed to `frame_id` via TF if necessary.

## Published Topics
- `path` (`nav_msgs/msg/Path`) — Planned path from the robot to the goal

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `frame_id` | `odom` | TF frame ID for the occupancy grid, odometry, goal, and published path |
| `max_unknown_forward_m` | `2.0` | How far forward of the robot (m) unknown cells are treated as traversable by A* |
| `max_unknown_sideways_m` | `1.0` | How far sideways of the robot (m) unknown cells are treated as traversable by A* |
| `line_of_sight_step_m` | `0.05` | Step size (m) for sampling segments during string-pulling line-of-sight checks. Smaller values give more accurate collision checking at the cost of more computation |
