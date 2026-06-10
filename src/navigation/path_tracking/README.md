# path_tracking
Path tracking for mobile robots. Subscribes to a planned path and odometry, and publishes velocity commands. Supports 
multiple controllers selectable via config. See each controller's README for algorithm details and tuning guidance.

## Controllers
| Algorithm | README |
|---|---|
| `pure_pursuit` | [pure_pursuit/README.md](path_tracking/controllers/pure_pursuit/README.md) |
| `stanley` | [stanley/README.md](path_tracking/controllers/stanley/README.md) |
| `differential_drive` | [differential_drive/README.md](path_tracking/controllers/differential_drive/README.md) |

## Mission Control Integration
The node reads the latched `mission_state` topic published by `autonav_mission_control` and adjusts the controller 
output accordingly:

- **Ramp slowdown**: while `in_ramp_approach` is set, the commanded forward speed is capped at `ramp_max_speed_mps` so 
  the robot climbs the ramp slowly and predictably.
- **Mission complete**: once `mission_complete` is set, no velocity commands are published and the robot stops.

If no mission state has been received (e.g. in modes where mission control is not running), commands are published 
unmodified.

## Subscribed Topics
- `odom` (`nav_msgs/msg/Odometry`) - Robot pose and velocity in the odometry frame
- `path` (`nav_msgs/msg/Path`) - Planned path to follow
- `mission_state` (`maverick_msgs/msg/MissionState`) - Mission state (latched); see Mission Control Integration

## Published Topics
- `nav_cmd_vel` (`geometry_msgs/msg/Twist`) — Velocity command for the robot base

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `algorithm` | `str` | `"differential_drive"` | Which controller to run. One of `"pure_pursuit"`, `"stanley"`, `"differential_drive"`. |
| `control_period_s` | `float` | `0.01` | Period of the control loop timer (s). |
| `base_frame_id` | `str` | `"base_link"` | Frame ID of the robot base, used to validate the child frame of incoming odometry. |
| `odom_frame_id` | `str` | `"odom"` | Frame ID of the odometry frame, used to validate incoming odometry and path messages. |
| `ramp_max_speed_mps` | `float` | `1.0` | Maximum forward speed (m/s) while `in_ramp_approach` is set. |
