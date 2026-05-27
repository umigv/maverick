# Pure Pursuit Controller
Geometric path tracking controller that steers toward a moving lookahead point on the path.

At each control step, the controller finds a point on the path a fixed distance (the lookahead distance) ahead of the
robot, then computes the constant-curvature arc that passes through that point. The required angular velocity follows
directly from the arc curvature and the commanded linear speed.

The lookahead distance is adaptive: it grows linearly with speed (via `lookahead_speed_gain`) and is clamped to
`[min_lookahead_distance_m, max_lookahead_distance_m]`. A larger lookahead produces smoother but wider turns; a smaller
lookahead tracks more tightly but can oscillate.

When no lookahead circle intersection exists (e.g. the robot has deviated far from the path), the controller falls back
to projecting the robot onto the path and advancing from there.

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `max_angular_speed_radps` | `0.6` | Maximum angular velocity command (rad/s). Linear and angular are scaled down together to respect this limit |
| `base_lookahead_distance_m` | `0.8` | Lookahead distance when stationary (m) |
| `min_lookahead_distance_m` | `0.8` | Minimum clamped lookahead distance (m) |
| `max_lookahead_distance_m` | `1.5` | Maximum clamped lookahead distance (m) |
| `lookahead_speed_gain` | `0.0` | Scales current speed to extend lookahead distance at higher speeds |
| `linear_speed_gain` | `0.5` | Scales lookahead distance to produce the forward speed command (m/s per m) |
