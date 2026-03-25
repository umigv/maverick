# autonav_goal_selection
GPS waypoint following with local goal selection for autonomous navigation.

Loads a list of GPS waypoints from a JSON file, converts them to map-frame coordinates via the `fromLL` service, and 
selects a local goal from the occupancy grid on a timer. The goal is chosen by scoring every drivable cell with a 
heuristic. Waypoints are advanced automatically as the robot reaches them.

## Subscribed Topics
- `odom` (`nav_msgs/msg/Odometry`) — Robot pose in the world frame, used for waypoint advancement and goal scoring
- `occupancy_grid` (`nav_msgs/msg/OccupancyGrid`) — Occupancy grid used to find drivable goal candidates

## Published Topics
- `goal` (`geometry_msgs/msg/PointStamped`) — Selected local goal in the world frame, published at `goal_publish_period_s`
- `gps_waypoint` (`geometry_msgs/msg/PointStamped`) — Current GPS waypoint converted to map-frame coordinates; latched

The `gps_waypoint` topic uses a non-default QoS profile:
| Setting | Value | Effect |
|---|---|---|
| Durability | `TRANSIENT_LOCAL` | Late-joining subscribers immediately receive the current waypoint |
| History | `KEEP_LAST` (depth 1) | Only the most recent waypoint is retained |

Subscribers to `gps_waypoint` **must use a compatible QoS** (`TRANSIENT_LOCAL`) or they will not receive the latched
message on connect. In code, use `utils.qos.LATCHED`. In RViz, set the topic's **Durability Policy** to `Transient Local`.

## Services
- `fromLL` (`robot_localization/srv/FromLL`) — Called at startup to convert each GPS waypoint to a map-frame point

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `goal_selection_params` | `GoalSelectionParams` | required | Parameters for the goal selection heuristic (see below) |
| `waypoints_file_path` | `Path` | required | Path to a JSON file containing the list of GPS waypoints to navigate |
| `goal_publish_period_s` | `float` | `5.0` | How often (s) to publish a new local goal |
| `waypoint_reached_threshold` | `float` | `1.0` | Distance (m) within which a waypoint is considered reached |
| `map_frame_id` | `str` | `map` | TF frame ID for the map coordinate frame |
| `world_frame_id` | `str` | `odom` | TF frame ID for the world coordinate frame |

### Goal Selection Parameters (`goal_selection_params`)
(Currently empty)