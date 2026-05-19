# autonav_goal_selection
Map waypoint following with local goal selection for autonomous navigation.

Loads a list of map-frame waypoints from a JSON file and, on a timer, selects a local goal in the occupancy grid by
casting rays across the robot's forward arc. The longest, most waypoint-aligned drivable ray is chosen; its endpoint
(pulled in by a safety margin) is published as the local goal. Waypoints are advanced automatically as the robot
reaches them.

## Subscribed Topics
- `odom` (`nav_msgs/msg/Odometry`) — Robot pose in the world frame, used for waypoint advancement and ray casting
- `occupancy_grid` (`nav_msgs/msg/OccupancyGrid`) — Local occupancy grid the rays are cast through

## Published Topics
- `goal` (`geometry_msgs/msg/PointStamped`) — Selected local goal in the world frame, published at `goal_publish_period_s`
- `gps_waypoint` (`geometry_msgs/msg/PointStamped`) — Current map-frame waypoint; latched
- `goal_selection_debug` (`visualization_msgs/msg/MarkerArray`) — Per-tick debug rendering of the rays, the chosen
  endpoint, and the waypoint direction. Only published when `publish_debug` is true.

The `gps_waypoint` topic uses a non-default QoS profile:
| Setting | Value | Effect |
|---|---|---|
| Durability | `TRANSIENT_LOCAL` | Late-joining subscribers immediately receive the current waypoint |
| History | `KEEP_LAST` (depth 1) | Only the most recent waypoint is retained |

Subscribers to `gps_waypoint` **must use a compatible QoS** (`TRANSIENT_LOCAL`) or they will not receive the latched
message on connect. In code, use `utils.qos.LATCHED`. In RViz, set the topic's **Durability Policy** to `Transient Local`.

## Waypoints File Format
Map-frame x/y values, in meters:
```json
{
    "waypoints": [
        {"x": 5.0, "y": 0.0},
        {"x": 5.0, "y": 5.0}
    ]
}
```
GPS (lat/lon) parsing is not currently supported but may be reintroduced behind a format field.

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `goal_selection_params` | `GoalSelectionParams` | required | Parameters for the ray-cast goal selection (see below) |
| `waypoints_file_path` | `Path` | required | Path to a JSON file containing the list of map-frame waypoints |
| `goal_publish_period_s` | `float` | `0.5` | How often (s) to run goal selection and publish a new goal |
| `waypoint_reached_threshold` | `float` | `1.0` | Distance (m) within which a waypoint is considered reached |
| `map_frame_id` | `str` | `map` | TF frame ID for the map coordinate frame |
| `world_frame_id` | `str` | `odom` | TF frame ID for the world coordinate frame |
| `publish_debug` | `bool` | `false` | When true, publish the `goal_selection_debug` MarkerArray |

### Goal Selection Parameters (`goal_selection_params`)
| Parameter | Type | Default | Description |
|---|---|---|---|
| `num_rays` | `int` | `37` | Number of rays cast across the forward arc (~5° spacing at default) |
| `arc_half_angle_rad` | `float` | `π/2` | Half-angle of the forward arc; rays span `[-h, +h]` from the heading |
| `max_ray_length_m` | `float` | `2.4` | Maximum free length walked along each ray |
| `step_size_m` | `float` | `0.05` | Step size used when walking each ray; should be ≈ grid resolution |
| `alignment_exponent` | `float` | `2.0` | Sharpness of the multiplicative alignment factor `((1 + cos(angle_to_waypoint)) / 2) ** exp`. 0 disables waypoint bias; larger values penalize off-axis rays harder. Anti-aligned rays always score 0. |
| `min_goal_progress_m` | `float` | `0.2` | If the winning ray's free length is below this, no goal is published (treat as stuck). Set to 0 to publish whatever the best ray is. |
| `neighbor_smoothing_window` | `int` | `1` | Number of neighbors per side averaged into each ray's score |
| `safety_margin_m` | `float` | `0.1` | Distance (m) the chosen endpoint is pulled back from the ray terminator |
| `max_unknown_forward_m` | `float` | `2.0` | How far forward unknown cells are treated as drivable (mirrors path_planning) |
| `max_unknown_sideways_m` | `float` | `1.0` | How far sideways unknown cells are treated as drivable (mirrors path_planning) |

## Debug Visualization
With `publish_debug: true`, subscribe to `goal_selection_debug` in RViz (MarkerArray display). You will see:
- Thin lines for every cast ray, colored red→green by score
- Spheres at each ray's terminator, colored by why the walk stopped: blue (max length), red (obstacle), yellow
  (unknown out of bounds)
- A thick white line for the chosen ray
- A green sphere at the chosen goal
- A blue arrow from the robot to the current waypoint
