# autonav_mission_control
Global mission state tracking for autonomous navigation: waypoint sequencing, no man's land zones, ramp approach, and 
lane detection gating.

## How it works

### The big picture
This node is the single source of truth for mission state. It loads an ordered list of waypoints from a JSON file, 
tracks which waypoint the robot is currently targeting, and publishes a latched `mission_state` message on every state 
transition. Other nodes subscribe to `mission_state` and adjust their behavior accordingly (e.g. disabling lane 
detection inside no man's land, slowing down on ramp approach).

### Waypoint sequencing
On each odometry message, the node computes the distance from the robot to the current waypoint. When that distance 
falls below `waypoint_reached_threshold_m`, the waypoint is marked reached and the index advances to the next one. When 
the final waypoint is reached, a one-shot timer starts and `mission_complete` is set after `mission_complete_delay_s` 
seconds. During that window, `current_waypoint` is a far point (10 km) projected along the robot's heading at the 
moment the final waypoint was reached, so normal goal selection keeps the robot driving straight ahead until the flag 
flips. When the timer fires, the node publishes the final state and exits; the autonav launch watches for this exit 
and shuts down the rest of the stack.

### No man's land
Certain waypoints are flagged `no_mans_land_enter` or `no_mans_land_exit`. Reaching an enter waypoint sets 
`in_no_mans_land = true`; reaching an exit waypoint clears it. The course file is validated at startup to ensure enters 
and exits are balanced and properly sequenced. Any violation is fatal.

Inside no man's land, `lane_detection_enabled` is normally `false` to suppress lane detection (lane markings are absent 
or unreliable in this zone). The exception: when the robot comes within `lane_detection_enable_near_exit_radius_m` of 
the next exit waypoint, `lane_detection_enabled` is re-enabled so the robot can see lane markings as it approaches the 
zone boundary.

### Ramp approach
Waypoints flagged `ramp_approach` activate `in_ramp_approach` when the robot enters a radius of `ramp_approach_radius_m`
around them. This lets downstream nodes (e.g. speed controller) switch to a slower, more careful mode before hitting the
ramp. The flag is cleared when the waypoint is reached.

### Recovery
External nodes can call the `request_recovery` service to set `in_recovery = true` and `recovery_complete` to clear it.
The node publishes state on each transition so subscribers react immediately.

## Waypoints File Format
```json
{
    "waypoints": [
        {"x": 5.0, "y": 0.0},
        {"latitude": 42.2946, "longitude": -83.7238},
        {"x": 20.0, "y": 0.0, "no_mans_land_enter": true},
        {"x": 40.0, "y": 0.0, "no_mans_land_exit": true},
        {"x": 50.0, "y": 10.0, "ramp_approach": true}
    ]
}
```

Each waypoint is either map-frame `x`/`y` (meters) or GPS `latitude`/`longitude` (converted via `fromLL` at startup).
The flags are mutually exclusive per waypoint: a single waypoint cannot have both `no_mans_land_enter` and 
`no_mans_land_exit` set. The course file is validated at startup and the node exits with a fatal error if pairs are 
unbalanced or out of order.

## Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `odom` | `nav_msgs/msg/Odometry` | Robot pose used to compute distance to waypoints |

## Published Topics
| Topic | Type | Description |
|---|---|---|
| `mission_state` | `maverick_msgs/msg/MissionState` | Latched mission state; published on every state transition |

`mission_state` is latched: publisher and subscribers must both use `utils.qos.LATCHED` — see the
[utils README](../../core/utils/README.md#utilsqos).

### MissionState fields
See [`MissionState.msg`](../../core/maverick_msgs/msg/MissionState.msg) for the field definitions and what each one
means for subscribers.

## Services
| Service | Type | Description |
|---|---|---|
| `request_recovery` | `std_srvs/srv/Trigger` | Sets `in_recovery = true` and publishes state |
| `recovery_complete` | `std_srvs/srv/Trigger` | Clears `in_recovery` and publishes state |

## Service Clients
| Service | Type | Description |
|---|---|---|
| `fromLL` | `robot_localization/srv/FromLL` | Converts GPS waypoints to map-frame coordinates at startup |

## Adding new state fields
There are two kinds of state in this node, and which kind a new field is determines where it belongs:

- **Polled state** can be recomputed from scratch at any moment from current values (pose, waypoints, config, other 
  state). It belongs in the compute-and-apply loop in `update_mission_state`, which samples it on every odometry 
  message, diffs it against the stored value, and publishes on change. Compute functions must be pure reads — the loop 
  owns the write.
- **Event-driven state** changes at a moment in time — a service call arrives, a timer fires. There is nothing to 
  poll; the event itself is the change. The event callback sets the attribute and calls `publish_state()` directly. 
  `in_recovery` (service-driven) and `mission_complete` (timer-driven) work this way.

The `on_change` hook bridges the two: when a polled transition should kick off event-driven machinery, the hook is the 
place to start it (e.g. reaching the final waypoint starts the mission-complete timer).

To add a polled field: add the attribute in `__init__`, write a pure compute function returning 
`(new_value, log_message | None)`, register it as a `StateUpdate` in the updates list in `update_mission_state`, add 
the field to `MissionState.msg` (and rebuild `maverick_msgs`), and set it in `publish_state`. Follow the existing 
fields in [`autonav_mission_control.py`](autonav_mission_control/autonav_mission_control.py): `in_ramp_approach` is the 
simplest template, and `current_waypoint_index` shows an `on_change` side effect (starting the mission-complete timer).
