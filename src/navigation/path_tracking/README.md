# path_tracking
Path tracking for mobile robots. Supports pure pursuit and Stanley controllers, selectable via config.

Subscribes to a planned path and odometry, and publishes velocity commands.

## Subscribed Topics
- `odom` (`nav_msgs/msg/Odometry`) - Robot pose and velocity in the odometry frame
- `path` (`nav_msgs/msg/Path`) - Planned path to follow

## Published Topics
- `nav_cmd_vel` (`geometry_msgs/msg/Twist`) — Velocity command for the robot base

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `algorithm` | `str` | `"stanley"` | Which controller to run. One of `"pure_pursuit"`, `"stanley"`. |
| `control_period_s` | `float` | `0.01` | Period of the control loop timer (s). |
| `base_frame_id` | `str` | `"base_link"` | Frame ID of the robot base, used to validate the child frame of incoming odometry. |
| `odom_frame_id` | `str` | `"odom"` | Frame ID of the odometry frame, used to validate incoming odometry and path messages. |

### `pure_pursuit`
| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_angular_speed_radps` | `float` | `0.6` | Maximum angular velocity command (rad/s). Commands are scaled down to stay within this limit. |
| `base_lookahead_distance_m` | `float` | `0.8` | Lookahead distance when stationary (m). Added to the speed-dependent term. |
| `min_lookahead_distance_m` | `float` | `0.8` | Minimum clamped lookahead distance (m). |
| `max_lookahead_distance_m` | `float` | `1.5` | Maximum clamped lookahead distance (m). |
| `lookahead_speed_gain` | `float` | `0.0` | Gain applied to current speed when computing adaptive lookahead distance. |
| `linear_speed_gain` | `float` | `0.5` | Gain applied to lookahead distance to produce the linear velocity command. |

### `stanley`
| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_speed_mps` | `float` | `0.6` | Reference forward speed (m/s). |
| `cross_track_gain` | `float` | `0.8` | Gain on cross-track error in the Stanley steering law. |
| `front_offset_m` | `float` | `0.5` | Distance ahead of `base_link` where the virtual front axle is placed (m). Also used as the virtual wheelbase for converting steering angle to angular velocity. |
| `max_steer_rad` | `float` | `1.2` | Saturation limit on the steering angle (rad). |
| `max_angular_speed_radps` | `float` | `1.2` | Maximum angular velocity command (rad/s). |
| `goal_tolerance_m` | `float` | `0.3` | Stop when the front point is within this distance of the final path point (m). |
| `max_lateral_accel_mps2` | `float` | `1.0` | Lateral acceleration ceiling used to cap speed in curved sections (m/s²). |
| `curvature_lookahead_m` | `float` | `0.5` | Arclength ahead of the projection over which heading change is accumulated for curvature-based speed limiting (m). |
