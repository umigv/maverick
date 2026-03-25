# State Machine
This package provides a State Machine that tracks and publishes the robot’s high-level operating state. The state 
machine exposes services for state transitions and publishes the robot state to a topic for other nodes to subscribe 
to and react accordingly.

## States
There are three mutually exclusive states:
- **`normal`** – default operating mode  
- **`no_mans_land`** – no mans land-specific behavior enabled  
- **`recovery`** – recovery / fault handling mode  

1. State priority is fixed and deterministic: recovery > no_mans_land > normal
2. States are published only on state changes
3. On startup, the node immediately publishes the initial state (`normal`)
4. Late-joining subscribers immediately receive the most recent state

## Published Topics
- `state` (`std_msgs/String`) - The current robot state as one of `"normal"`, `"no_mans_land"`, or `"recovery"`

The `state` topic uses a non-default QoS profile:
| Setting | Value | Effect |
|---|---|---|
| Reliability | `RELIABLE` | Guaranteed delivery |
| Durability | `TRANSIENT_LOCAL` | Late-joining subscribers immediately receive the last published state |
| History | `KEEP_LAST` (depth 1) | Only the most recent message is retained |

Subscribers to `state` **must use a compatible QoS** (`RELIABLE` + `TRANSIENT_LOCAL`) or they will not receive the
latched message on connect. A subscriber with default QoS (`BEST_EFFORT` or `VOLATILE`) will silently receive nothing. 
In code, use `utils.qos.LATCHED`. In RViz, set the topic's **Durability Policy** to `Transient Local`.

## Services
- `state/set_no_mans_land` (`std_srvs/SetBool`) - Enable or disable no_mans_land mode
- `state/set_recovery` (`std_srvs/SetBool`) - Enable or disable recovery mode

## Example Usage
```bash
ros2 topic echo /state # Observe current state
ros2 service call /state/set_no_mans_land std_srvs/srv/SetBool "{data: true}" # Enable no_mans_land mode
ros2 service call /state/set_recovery std_srvs/srv/SetBool "{data: true}" # Enter recovery mode (overrides no_mans_land)
ros2 service call /state/set_recovery std_srvs/srv/SetBool "{data: false}" # Exit recovery mode
ros2 service call /state/set_no_mans_land std_srvs/srv/SetBool "{data: false}" # Disable no_mans_land mode
```
