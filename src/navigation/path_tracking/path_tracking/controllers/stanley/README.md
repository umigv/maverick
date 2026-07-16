# stanley

Front-axle path tracking controller that jointly minimizes heading error and cross-track error.

The controller evaluates errors at a virtual front axle placed `front_offset_m` ahead of the robot. The steering command combines two terms: a heading error term (aligning the robot's heading with the path tangent) and a cross-track term (driving the front axle toward the path via an arctangent law that saturates at high speeds). The combined steering angle is mapped to angular velocity using a bicycle-model approximation with `front_offset_m` as the wheelbase.

Forward speed is capped before corners by a lateral-acceleration budget: the controller looks `curvature_lookahead_m` ahead, accumulates total heading change, estimates the turn radius, and reduces speed so that `v² / r ≤ max_lateral_accel_mps2`. This slows the robot *before* it reaches the corner rather than reacting to it.
