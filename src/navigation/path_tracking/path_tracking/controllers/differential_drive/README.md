# differential_drive

Unicycle path tracking controller designed for differential-drive robots, combining heading and cross-track error feedback with an adaptive lookahead for corner handling.

At each control step, the robot is projected onto the nearest path segment. Two error terms are computed: heading error (angle between the robot's heading and the path tangent) and cross-track error (signed lateral distance from the path). The angular velocity command is a weighted sum of both:

```
omega = kp_heading * theta_err + kp_cross * cos(theta_err) * cte
```

The `cos(theta_err)` factor on the cross-track term fades it out when the robot is facing away from the path, preventing over-correction. Forward speed is reduced proportionally to `cos(theta_err)` and further capped so that lateral drift (`v * sin(theta_err)`) never exceeds `max_lateral_speed_mps`.

**Adaptive lookahead:** Near corners, the heading reference shifts from the local segment tangent to a point `heading_lookahead_m` ahead on the path. This anticipates the turn and reduces heading error overshoot at the apex. The lookahead ramps in as the robot approaches a corner vertex and ramps back out after passing it; on straight segments it stays zero so the local tangent is used directly.
