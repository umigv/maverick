# Stanley Controller
Front-axle path tracking controller that jointly minimizes heading error and cross-track error.

The controller evaluates errors at a virtual front axle placed `front_offset_m` ahead of the robot. The steering command 
combines two terms: a heading error term (aligning the robot's heading with the path tangent) and a cross-track term 
(driving the front axle toward the path via an arctangent law that saturates at high speeds). The combined steering
angle is mapped to angular velocity using a bicycle-model approximation with `front_offset_m` as the wheelbase.

Forward speed is capped before corners by a lateral-acceleration budget: the controller looks `curvature_lookahead_m`
ahead, accumulates total heading change, estimates the turn radius, and reduces speed so that `v² / r ≤
max_lateral_accel_mps2`. This slows the robot *before* it reaches the corner rather than reacting to it.

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `target_speed_mps` | `1.35` | Maximum forward speed (m/s). Reduced automatically before corners |
| `cross_track_gain` | `0.6` | Gain on cross-track error in the Stanley steering law |
| `front_offset_m` | `0.85` | Distance ahead of base_link where the virtual front axle is placed (m). Also used as the wheelbase in the bicycle model |
| `max_steer_rad` | `1.2` | Saturation limit on the steering angle command (rad) |
| `max_angular_speed_radps` | `1.0` | Hard cap on angular velocity after bicycle-model conversion (rad/s) |
| `goal_tolerance_m` | `0.3` | Stop when the front axle is within this distance of the final path point (m) |
| `max_lateral_accel_mps2` | `1.0` | Lateral acceleration budget used to cap speed before corners (m/s²) |
| `curvature_lookahead_m` | `1.5` | Arclength ahead over which heading change is accumulated for speed limiting (m) |
