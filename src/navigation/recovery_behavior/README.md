# recovery_behavior

Mission-control-driven recovery node. When `in_recovery` becomes true the robot sweeps left/right using an ultrasonic sensor to find a clear path, then backs up. When the maneuver finishes it calls `recovery_complete` so mission control can clear `in_recovery`.

Reads ultrasonic distance from `/dev/ultrasonic` at 9600 baud.

## Subscribed Topics
- `mission_state` (`maverick_msgs/MissionState`) — triggers recovery when `in_recovery` transitions to true

## Published Topics
- `recovery_cmd_vel` (`geometry_msgs/Twist`)

## Service Clients
- `recovery_complete` (`std_srvs/Trigger`) — signals recovery end back to mission control
