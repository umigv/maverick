# autonav_goal_selection

Local goal selection toward the mission's current waypoint for autonomous navigation.

## How it works

### The big picture

The mission is a list of coarse GPS/map waypoints (think: "go to this field corner, then that one") tracked by `autonav_mission_control`, which publishes the current target waypoint on the latched `mission_state` topic. The waypoints are far apart and don't account for obstacles between them. This node's job is to bridge that gap: every publish period it picks a local goal a few meters ahead that keeps the robot moving toward the current waypoint while staying in drivable space.

### Step 1 - Ray casting

Each tick, the node shoots a fan of rays outward from the robot's current position. The fan spans `arc_angle_rad` centered on the robot's heading, with rays spaced `ray_interval_rad` apart.

Each ray is walked step by step (`step_size_m`) until it hits either:

- an **occupied cell** (obstacle), or
- an **unknown cell** too far outside the robot's forward path (unknown cells within `max_unknown_forward_m` forward and `max_unknown_sideways_m` sideways are treated as drivable, since the area right under the robot is blocked and marked as unknown even though it is drivable)

The ray's **length** is how far it traveled before stopping. A longer ray means more open space in that direction.

### Step 2 - Scoring

Each ray gets a score:

```
score = ray_length * momentum_factor
```

**`ray_length`** rewards pointing into open space.

**`momentum_factor`** is a value in [0, 1] that rewards pointing in roughly the same direction the robot has been going. It exists to prevent the robot from oscillating. Without it, a small asymmetry in the occupancy grid could flip the chosen direction every tick, causing the robot to wiggle rather than drive straight.

The momentum factor has two parts that interact:

- **Alignment term**: how well this ray aligns with the current momentum angle. Rays pointing along momentum score close to 1; rays pointing sideways or backward score closer to `alignment_floor`. The sharpness of this penalty is controlled by `alignment_gain`.
- **Obstacle scale**: if the momentum aligned ray itself is short (something is blocking the direction the robot has been going), the alignment penalty is relaxed so that longer escape route rays can win. This kicks in below `obstacle_threshold_m` and fades to zero (all rays equally weighted by length) as the momentum ray approaches zero.

After scoring, each ray's score is **neighbor smoothed** by averaging it with its `neighbor_smoothing_window` adjacent rays. This prevents a single unusually long gap in the occupancy grid from dominating, and makes the chosen direction more stable.

### Step 3 - Choosing and publishing the goal

The highest scoring ray wins. The goal point is placed along that ray at `ray_length - safety_margin_m` from the robot (pulling back from the termination point keeps the goal away from the obstacle edge).

If the winning ray is shorter than `min_goal_progress_m`, the robot is considered stuck. No goal is published and we trigger recovery

### Step 4 - Updating momentum

After choosing a direction, the stored **momentum angle** is nudged toward the chosen ray's angle using an exponential moving average (EMA):

```
momentum_angle += ema_alpha * (chosen_angle − momentum_angle)
```

Low `ema_alpha` means momentum changes slowly; high values make it respond faster to sharp turns.

### Waypoint approach

When the robot is within `waypoint_approach_radius_m` of the current waypoint, ray casting is skipped entirely and the waypoint itself is published as the goal. This ensures the robot reaches waypoints precisely rather than stopping short. The same bypass applies while `in_no_mans_land` or `in_ramp_approach` is set in the mission state. Waypoint advancement itself is handled by `autonav_mission_control`; this node always drives toward whatever `current_waypoint` the latest `mission_state` message contains.

## Mission Control Integration

This node reads mission state from the latched `mission_state` topic published by `autonav_mission_control` (see that package's README for the waypoints file format and state semantics).

**Recovery** is triggered when all rays are shorter than `min_goal_progress_m` (the robot is surrounded by obstacles). The node calls the `request_recovery` service, resets momentum, and stops publishing goals while `in_recovery` is set. The recovery behavior node is responsible for driving the robot out of the stuck situation and calling `recovery_complete` when done.

Goal publishing also stops once `mission_complete` is set.

## Subscribed Topics

| Topic            | Type                         | Description                                                               |
| ---------------- | ---------------------------- | ------------------------------------------------------------------------- |
| `odom`           | `nav_msgs/Odometry`          | Robot pose in the world frame                                             |
| `occupancy_grid` | `nav_msgs/OccupancyGrid`     | Local occupancy grid rays are cast through                                |
| `mission_state`  | `maverick_msgs/MissionState` | Mission state (latched); provides the current waypoint and behavior flags |

## Published Topics

| Topic                  | Type                             | Description                                                    |
| ---------------------- | -------------------------------- | -------------------------------------------------------------- |
| `goal`                 | `geometry_msgs/PointStamped`     | Selected local goal in the world frame                         |
| `goal_selection_debug` | `visualization_msgs/MarkerArray` | Ray visualization; only published when `publish_debug` is true |

## Service Clients

| Service            | Type               | Description                                      |
| ------------------ | ------------------ | ------------------------------------------------ |
| `request_recovery` | `std_srvs/Trigger` | Triggers recovery when no drivable goal is found |

## Debug Visualization

With `publish_debug: true`, subscribe to `goal_selection_debug` in RViz (MarkerArray display). You will see:

- Thin lines for all rays, colored red→green by normalized score
- A thick yellow line for the chosen (highest-scoring) ray
- A thick blue line for the momentum-aligned ray
