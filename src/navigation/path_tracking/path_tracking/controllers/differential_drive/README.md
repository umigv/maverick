# Differential Drive Controller
Unicycle path tracking controller designed for differential-drive robots, combining heading and cross-track error
feedback with an adaptive lookahead for corner handling.

At each control step, the robot is projected onto the nearest path segment. Two error terms are computed: heading error
(angle between the robot's heading and the path tangent) and cross-track error (signed lateral distance from the path).
The angular velocity command is a weighted sum of both:

```
ω = kp_heading · θ_err + kp_cross · cos(θ_err) · cte
```

The `cos(θ_err)` factor on the cross-track term fades it out when the robot is facing away from the path, preventing
over-correction. Forward speed is reduced proportionally to `cos(θ_err)` and further capped so that lateral drift
(`v · sin(θ_err)`) never exceeds `max_lateral_speed_mps`.

**Adaptive lookahead:** Near corners, the heading reference shifts from the local segment tangent to a point
`heading_lookahead_m` ahead on the path. This anticipates the turn and reduces heading error overshoot at the apex. The
lookahead ramps in as the robot approaches a corner vertex and ramps back out after passing it; on straight segments it
stays zero so the local tangent is used directly.

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `target_speed_mps` | `1.35` | Maximum forward speed (m/s). Reduced by `cos(heading_error)` in practice |
| `kp_heading` | `2.0` | Proportional gain on heading error (rad/s per rad) |
| `kp_cross` | `1.5` | Proportional gain on cross-track error, scaled by `cos(heading_error)` (rad/s per m) |
| `max_angular_speed_radps` | `1.5` | Hard cap on angular velocity command (rad/s) |
| `max_lateral_speed_mps` | `0.2` | Maximum tolerated lateral drift speed (m/s). Caps forward speed to keep `v·sin(θ) ≤` this value |
| `heading_lookahead_m` | `1.0` | Distance ahead used for the heading reference near corners (m). Set to `0` to always use the local tangent |
| `goal_tolerance_m` | `0.3` | Stop when the robot center is within this distance of the final path point (m) |
