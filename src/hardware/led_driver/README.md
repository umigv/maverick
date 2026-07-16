# led_driver

Drives the safety-light LED strip over serial based on the robot's current e-stop value, teleop activity, and mission state. Waits up to `ready_timeout_s` for the Arduino to send `READY` before starting.

## Read Files

- `estop_file_path` - E-stop state written by `estop_driver`

## Subscribed Topics

- `teleop_cmd_vel` (`geometry_msgs/TwistStamped`) - Marks teleop as active for `cmd_vel_timeout_s` after the last message
- `nav_cmd_vel` (`geometry_msgs/TwistStamped`) - Marks the nav stack as active for `cmd_vel_timeout_s` after the last message
- `mission_state` (`maverick_msgs/MissionState`) - Current mission state, latched

## LED Priority

| Priority | Condition                                                           | Code | Effect          |
| -------- | ------------------------------------------------------------------- | ---- | --------------- |
| 1        | e-stop file is `"1"`                                                | `6`  | Flashing red    |
| 2        | `teleop_cmd_vel` received within `cmd_vel_timeout_s`                | `1`  | Solid blue      |
| 3        | `mission_complete`                                                  | `7`  | Solid green     |
| 4        | `in_recovery`                                                       | `4`  | Flashing yellow |
| 5        | `in_ramp_approach`                                                  | `5`  | Flashing purple |
| 6        | `in_no_mans_land`                                                   | `3`  | Flashing green  |
| 7        | mission state received                                              | `2`  | Flashing blue   |
| 8        | no mission state, `nav_cmd_vel` received within `cmd_vel_timeout_s` | `2`  | Flashing blue   |
| 9        | no mission state, nav stack inactive                                | `9`  | Rainbow         |
