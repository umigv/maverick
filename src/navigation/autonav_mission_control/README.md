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
| Field | Type | Description |
|---|---|---|
| `in_recovery` | `bool` | Robot is in a recovery maneuver |
| `in_ramp_approach` | `bool` | Robot is within `ramp_approach_radius_m` of a ramp waypoint |
| `in_no_mans_land` | `bool` | Robot is between a no man's land enter and exit waypoint |
| `lane_detection_enabled` | `bool` | Lane detection should be active |
| `mission_complete` | `bool` | Set `mission_complete_delay_s` seconds after the final waypoint is reached |
| `current_waypoint` | `geometry_msgs/PointStamped` | Map-frame position of the current target waypoint; after the final waypoint, a far point projected along the robot's heading |

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

State is driven by a compute-and-apply loop in `update_mission_state`. Each entry in the loop is a `StateUpdate`:

```python
class StateUpdate(NamedTuple):
    attr: str  # name of the state attribute on AutonavMissionControl
    compute: Callable[[], tuple[Any, str | None]]  # returns (new value, optional log message)
    on_change: Callable[[Any, Any], None] | None = None  # called with (old value, new value)
```

To add a new state field:

**1. Add the field to `__init__`:**
```python
self.my_flag: bool = False
```

**2. Write a compute function** that returns `(new_value, log_message | None)`. The function should read from `self` 
freely but never write to it. All state writes happen in the loop after all values are computed:
```python
def compute_my_flag(self) -> tuple[bool, str | None]:
    assert self.robot_pose is not None
    new_val = ...  # derive from self.robot_pose, self.waypoints, self.config, etc.
    return new_val, "My flag activated" if new_val else None
```

**3. Add it to the updates list** in `update_mission_state`:
```python
StateUpdate("my_flag", self.compute_my_flag),
```

**4. Add the field to the message definition** in `maverick_msgs/msg/MissionState.msg` (with a comment describing what 
it means for subscribers), then rebuild `maverick_msgs` so the updated message is generated.

**5. Publish it** by setting the new field on the `MissionState` message in `publish_state`.

### On-change callbacks
If you need a side effect when a field transitions (e.g. starting a timer, calling a service), pass an `on_change` 
callback (defaults to `None`). The callback receives `(old_value, new_value)` and is called after logging but before 
the new value is written:

```python
def _on_my_flag_change(self, old: bool, new: bool) -> None:
    if not old and new:  # False → True transition
        ...

# in the updates list:
StateUpdate("my_flag", self.compute_my_flag, on_change=self._on_my_flag_change),
```

For example, if you want to set a flag for 20 seconds after exiting no man's land (i.e. change goal selection behavior),
you can add a change function to `compute_no_mans_land`, set the flag and create a ros timer for 20 seconds, then clear
the flag within the timer. 

## Config Parameters (`AutonavMissionControlConfig`)
| Parameter | Type | Default | Description |
|---|---|---|---|
| `waypoints_file_path` | `Path` | required | Path to the JSON waypoints file |
| `waypoint_reached_threshold_m` | `float` | `0.5` | Distance (m) within which a waypoint is considered reached |
| `ramp_approach_radius_m` | `float` | `12.0` | Distance (m) from a `ramp_approach` waypoint at which `in_ramp_approach` activates |
| `lane_detection_enable_near_exit_radius_m` | `float` | `3.0` | Distance (m) from the next `no_mans_land_exit` waypoint within which lane detection is re-enabled |
| `mission_complete_delay_s` | `float` | `10.0` | Time (s) after the final waypoint is reached before `mission_complete` is set |
| `map_frame_id` | `str` | `"map"` | TF frame that odometry must be published in |
