# autonav_goal_selection
Map waypoint following with local goal selection for autonomous navigation.

## How it works

### The big picture
The robot is given a list of coarse GPS/map waypoints (think: "go to this field corner, then that one"). The waypoints
are far apart and don't account for obstacles between them. This node's job is to bridge that gap: every publish period 
it picks a local goal a few meters ahead that keeps the robot moving toward the current waypoint while staying in 
drivable space.

### Step 1 — Ray casting
Each tick, the node shoots a fan of rays outward from the robot's current position. The fan spans `arc_angle_rad`
(default 180°) centered on the robot's heading, with rays spaced `ray_interval_rad` (default ~1.25°) apart.

Each ray is walked step by step (`step_size_m`, default 5 cm ≈ grid resolution) until it hits either:
- an **occupied cell** (obstacle), or
- an **unknown cell** too far outside the robot's forward path (unknown cells within `max_unknown_forward_m` forward and
  `max_unknown_sideways_m` sideways are treated as drivable, since the area right under the robot is blocked and marked
  as unknown even though it is drivable)

The ray's **length** is how far it traveled before stopping. A longer ray means more open space in that direction.

### Step 2 — Scoring
Each ray gets a score:

```
score = ray_length * momentum_factor
```

**`ray_length`** rewards pointing into open space.

**`momentum_factor`** is a value in [0, 1] that rewards pointing in roughly the same direction the robot has been going. 
It exists to prevent the robot from oscillating. Without it, a small asymmetry in the occupancy grid could flip the 
chosen direction every tick, causing the robot to wiggle rather than drive straight.

The momentum factor has two parts that interact:
- **Alignment term**: how well this ray aligns with the current momentum angle. Rays pointing along momentum score close 
  to 1; rays pointing sideways or backward score closer to `alignment_floor` (default 0.1). The sharpness of this 
  penalty is controlled by `alignment_gain`.
- **Obstacle scale**: if the momentum aligned ray itself is short (something is blocking the direction the robot has
  been going), the alignment penalty is relaxed so that longer escape route rays can win. This kicks in below
  `obstacle_threshold_m` and fades to zero (all rays equally weighted by length) as the momentum ray approaches zero.

After scoring, each ray's score is **neighbor smoothed** by averaging it with its `neighbor_smoothing_window` adjacent
rays. This prevents a single unusually long gap in the occupancy grid from dominating, and makes the chosen direction
more stable.

### Step 3 — Choosing and publishing the goal
The highest scoring ray wins. The goal point is placed along that ray at `ray_length - safety_margin_m` from the robot 
(pulling back from the termination point keeps the goal away from the obstacle edge).

If the winning ray is shorter than `min_goal_progress_m`, the robot is considered stuck. No goal is published and we 
trigger recovery

### Step 4 — Updating momentum
After choosing a direction, the stored **momentum angle** is nudged toward the chosen ray's angle using an exponential
moving average (EMA):

```
momentum_angle += ema_alpha * (chosen_angle − momentum_angle)
```

Low `ema_alpha` (default 0.1) means momentum changes slowly; high values make it respond faster to sharp turns.

### Waypoint approach
When the robot is within `waypoint_approach_radius_m` (default 5 m) of the current waypoint, ray casting is skipped
entirely and the waypoint itself is published as the goal. This ensures the robot reaches waypoints precisely rather
than stopping short. Once within `waypoint_reached_threshold_m` (default 1 m), the waypoint is marked reached and the
next one becomes active.

## Waypoints File Format
```json
{
    "waypoints": [
        {"x": 5.0, "y": 0.0},
        {"latitude": 42.2946, "longitude": -83.7238},
        {"x": 10.0, "y": 5.0, "no_mans_land": true}
    ]
}
```
Each waypoint is either map frame `x`/`y` (meters) or GPS `latitude`/`longitude` (converted via `fromLL` at
startup). Setting `no_mans_land: true` toggles no man's land state when that waypoint is reached.

## State Machine Integration
This node both reads and writes state via the `state` topic and two service clients.

**Recovery** is triggered when all rays are shorter than `min_goal_progress_m` (the robot is surrounded by obstacles). 
The node calls `state/set_recovery true`, resets momentum, and stops publishing goals. The external state machine is 
responsible for driving the robot out of the stuck situation and calling `state/set_recovery false` when done.

**No-man's land** is toggled each time a waypoint flagged `no_mans_land: true` is reached. The node calls 
`state/set_no_mans_land` with the new boolean value and logs the transition. No man's land waypoints act as region 
boundary markers: the first one entering a region sets the flag true, the next one exiting sets it false.

## Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `odom` | `nav_msgs/msg/Odometry` | Robot pose in the world frame |
| `occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | Local occupancy grid rays are cast through |
| `state` | `std_msgs/msg/String` | State machine state; `"recovery"` suppresses goal publishing |

## Published Topics
| Topic | Type | Description |
|---|---|---|
| `goal` | `geometry_msgs/msg/PointStamped` | Selected local goal in the world frame |
| `waypoint` | `geometry_msgs/msg/PointStamped` | Current map-frame waypoint (latched) |
| `goal_selection_debug` | `visualization_msgs/msg/MarkerArray` | Ray visualization; only published when `publish_debug` is true |

The `waypoint` topic uses a latched QoS profile (`TRANSIENT_LOCAL`, depth 1). Late-joining subscribers receive the
current waypoint immediately on connect. Subscribers **must** use a compatible QoS (`TRANSIENT_LOCAL`) or they will
not receive the message. In code, use `utils.qos.LATCHED`. In RViz, set **Durability Policy** to `Transient Local`.

## Service Clients
| Service | Type | Description |
|---|---|---|
| `fromLL` | `robot_localization/srv/FromLL` | Converts GPS waypoints to map-frame coordinates at startup |
| `state/set_recovery` | `std_srvs/srv/SetBool` | Triggers recovery when no drivable goal is found |
| `state/set_no_mans_land` | `std_srvs/srv/SetBool` | Toggles no-man's-land state when a flagged waypoint is reached |

## Config Parameters (`AutonavGoalSelectionConfig`)
| Parameter | Type | Default | Description |
|---|---|---|---|
| `goal_selection_params` | `GoalSelectionParams` | required | Ray-cast goal selection parameters (see below) |
| `waypoints_file_path` | `Path` | required | Path to the JSON waypoints file |
| `goal_publish_period_s` | `float` | `0.25` | Timer period (s) for goal selection and publishing |
| `waypoint_reached_threshold_m` | `float` | `1.0` | Distance (m) within which a waypoint is considered reached |
| `waypoint_approach_radius_m` | `float` | `5.0` | Distance (m) from the waypoint within which ray-casting is bypassed and the waypoint is published directly |
| `map_frame_id` | `str` | `"map"` | TF frame for map-frame waypoint coordinates |
| `world_frame_id` | `str` | `"odom"` | TF frame for the world/robot pose coordinates |
| `publish_debug` | `bool` | `true` | When true, publish the `goal_selection_debug` MarkerArray |

### Goal Selection Parameters (`GoalSelectionParams`)
| Parameter | Type | Default | Description |
|---|---|---|---|
| `momentum` | `MomentumParams` | required | Momentum behavior parameters (see below) |
| `arc_angle_rad` | `float` | `π` | Full angular width of the forward arc; rays span symmetrically around the robot's heading |
| `ray_interval_rad` | `float` | `1.25°` | Angular spacing between rays; num_rays = `int(arc / interval) + 1` |
| `step_size_m` | `float` | `0.05` | Step size when walking each ray; should be ≈ grid resolution |
| `min_goal_progress_m` | `float` | `0.9` | Minimum ray length for a goal to be published; below this, no goal is published and recovery is triggered |
| `safety_margin_m` | `float` | `0.9` | Distance (m) the goal endpoint is pulled back from where the ray terminated |
| `neighbor_smoothing_window` | `int` | `2` | Number of neighbors per side averaged into each ray's score before picking |
| `max_unknown_forward_m` | `float` | `5.0` | How far forward unknown cells are treated as drivable |
| `max_unknown_sideways_m` | `float` | `2.5` | How far sideways unknown cells are treated as drivable |

### Momentum Parameters (`MomentumParams`)
Each tick the highest-scoring ray's angle is EMA-smoothed into a stored momentum angle. Scoring multiplies each ray's 
free length by a momentum factor in `[0, 1]` that rewards alignment with the momentum direction and scales down when the 
momentum-aligned ray itself is short (obstacle ahead).

Momentum factor formula, where $\theta$ is the angle between the ray and the momentum direction, $L_m$ is the free 
length of the momentum-aligned ray, and $\tau$, $f$, $g_o$, $g_a$ are `obstacle_threshold_m`, `alignment_floor`, 
`obstacle_gain`, `alignment_gain` respectively:

$$s = \min\!\left(1,\, \frac{L_m}{\tau}\right)^{g_o}$$

$$\text{factor} = (1 - s) + s \left[ f + (1 - f) \left(\frac{1 + \cos\theta}{2}\right)^{g_a} \right]$$

When $s = 1$ (momentum direction is clear), factor equals the alignment term. When $s = 0$ (momentum direction is 
blocked), factor equals 1 and all rays score equally on direction, letting the longest ray win.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `alignment_gain` | `float` | `2.0` | Sharpness of the directional penalty. `0` disables momentum bias; higher values penalize off-axis rays more aggressively |
| `alignment_floor` | `float` | `0.1` | Minimum alignment factor regardless of angle; prevents any ray from scoring zero due to direction alone |
| `obstacle_threshold_m` | `float` | `4.0` | Momentum ray length (m) below which momentum weight begins scaling down |
| `obstacle_gain` | `float` | `2.0` | Shape of the obstacle scaling curve: `<1` drops quickly, `1` is linear, `>1` stays near 1 until very close to 0 |
| `ema_alpha` | `float` | `0.1` | EMA smoothing factor for the momentum angle. Lower = more stable; higher = reacts faster to direction changes |

## Debug Visualization
With `publish_debug: true`, subscribe to `goal_selection_debug` in RViz (MarkerArray display). You will see:
- Thin lines for all rays, colored red→green by normalized score
- A thick yellow line for the chosen (highest-scoring) ray
- A thick blue line for the momentum-aligned ray
