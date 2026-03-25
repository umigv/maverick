# Sensor Simulator
Simulates encoder velocity, IMU, and INS sensors for testing the stack without real hardware.
Subscribes to `cmd_vel`, integrates velocity into a ground-truth pose, and publishes noisy sensor measurements that
mimic real hardware behavior.

Sensor noise is modeled using two components: per-sample **Gaussian white noise** for instantaneous measurement error,
and [**Ornstein-Uhlenbeck (OU) processes**](https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process) for
slow-drifting correlated errors (e.g. scale factor bias on encoders, multipath drift on GPS). OU processes are
mean-reverting, so errors stay bounded over time rather than growing without bound like a random walk.

## Behavior
- Integrates `cmd_vel` using 2D unicycle kinematics (Euler integration) at `update_period_s`
- Velocity is zeroed if no `cmd_vel` is received within `cmd_vel_timeout_s`
- Robot starts at map position `(0, 0)` with heading `initial_yaw_rad`
- All coordinates are ENU (East-North-Up): yaw=0 faces east, positive yaw is counter-clockwise

## Sensor Conventions

### Encoder Velocity
- Publishes `TwistWithCovarianceStamped` in the `base_frame_id` frame
- Applies OU multiplicative scale-factor drift to both linear and angular velocity proportional to speed so a
stationary robot always reads near zero and odom drift only accumulates while moving
- Applies per-sample Gaussian white noise on top of the drifting measurement
- Reported covariance is state-dependent: `var = noise_std² + (measurement × drift_std)²`

### VN-300 INS (IMU, GPS, INS velocity, INS odometry)
The VN-300 is modeled as a single sensor with one shared Kalman filter state per quantity, matching how the real
hardware works — all four outputs share the same OU drift states, so `gps`, `imu`, `ins_vel`, and `odom` stay
mutually consistent.

#### IMU
- Publishes `Imu` in the `imu_frame_id` frame
- Reports absolute orientation (yaw in ENU, relative to true north) and body-frame angular velocity z
- Applies shared OU yaw drift and shared OU velocity drift, plus per-sample Gaussian noise on each
- Looks up `base_frame_id` → `imu_frame_id` to apply the mounting offset to the robot's true yaw

#### GPS
- Publishes `NavSatFix` in the `ins_frame_id` frame (not the physical antenna)
- This is the Kalman Filter fused GPS output rather than tha raw antenna output
- Applies shared OU position drift and per-sample Gaussian noise in ENU
- ENU position is converted to WGS84 lat/lon/alt using a local topocentric projection anchored at `datum`

#### INS Velocity
- Publishes `TwistWithCovarianceStamped` in the `ins_frame_id` frame
- Reports body-frame linear.x and angular.z; shares OU velocity drift with `odom` twist and IMU angular velocity

#### INS Odometry
- Publishes `Odometry` in the `map` frame with `child_frame_id = ins_frame_id`
- Pose (position + orientation) uses the shared VN-300 position and yaw drift states
- Twist uses the shared VN-300 velocity drift states

## Subscribed Topics
- `cmd_vel` (`geometry_msgs/Twist`) - Velocity commands

## Published Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity in `base_frame_id`
- `imu` (`sensor_msgs/Imu`) - Simulated VN-300 IMU in `imu_frame_id`
- `gps` (`sensor_msgs/NavSatFix`) - Simulated VN-300 GPS fix in `ins_frame_id`
- `ins_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated VN-300 INS velocity in `ins_frame_id`
- `odom` (`nav_msgs/Odometry`) - Simulated VN-300 INS odometry in `map` frame
- `odom/ground_truth` (`nav_msgs/Odometry`) - Noiseless true pose in `map` frame (`child_frame_id = 
base_link_ground_truth`)

## Broadcasted TF
- `map` → `base_link_ground_truth` - Noiseless true robot pose, broadcast every update

## Required TF
- `base_link` → `imu_link` - IMU mounting orientation (used to compute yaw offset for IMU topic)

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `datum` | `list[float]` | required | ENU origin as `[latitude, longitude, altitude]` (deg, deg, m). Map `(0, 0, 0)` maps to this point. |
| `initial_yaw_rad` | `float` | `0.0` | Initial robot heading in true-north ENU map frame (rad) |
| `map_frame_id` | `str` | `map` | TF frame ID for the map frame |
| `base_frame_id` | `str` | `base_link` | TF frame ID for the robot base frame |
| `ground_truth_base_frame_id` | `str` | `base_link_ground_truth` | TF frame for the true robot pose |
| `cmd_vel_timeout_s` | `float` | `0.5` | Seconds without a cmd_vel before velocity is zeroed |
| `update_period_s` | `float` | `0.01` | Publish period for all sensors (s) |

### `enc_vel`
| Parameter | Type | Default | Description |
|---|---|---|---|
| `vx_noise_std_mps` | `float` | `0.0` | Per-sample Gaussian noise std on linear.x (m/s) |
| `wz_noise_std_radps` | `float` | `0.0` | Per-sample Gaussian noise std on angular.z (rad/s) |
| `vx_drift_std` | `float` | `0.0` | OU scale-factor steady-state std on linear.x (unitless) |
| `wz_drift_std` | `float` | `0.0` | OU scale-factor steady-state std on angular.z (unitless) |
| `drift_time_constant_s` | `float` | `30.0` | OU mean-reversion time constant for both scale factors (s) |

### `vn300`
| Parameter | Type | Default | Description |
|---|---|---|---|
| `imu_frame_id` | `str` | `imu_link` | TF frame ID for the IMU sensor |
| `ins_frame_id` | `str` | `base_link` | TF frame ID for the INS reference point (frame for GPS, ins_vel, odom) |
| `position_noise_std_m` | `float` | `0.0` | Per-sample Gaussian noise std on horizontal position (m) |
| `position_drift_std_m` | `float` | `0.0` | OU position-drift steady-state std (m). Set to `0` to disable. |
| `position_drift_time_constant_s` | `float` | `120.0` | OU mean-reversion time constant for position drift (s) |
| `vx_noise_std_mps` | `float` | `0.0` | Per-sample Gaussian noise std on linear.x (m/s) |
| `wz_noise_std_radps` | `float` | `0.0` | Per-sample Gaussian noise std on angular.z (rad/s) |
| `vx_drift_std_mps` | `float` | `0.0` | OU velocity-drift steady-state std on linear.x (m/s) |
| `wz_drift_std_radps` | `float` | `0.0` | OU drift steady-state std on angular.z (rad/s) |
| `vel_drift_time_constant_s` | `float` | `120.0` | OU mean-reversion time constant for velocity drift (s) |
| `yaw_noise_std_rad` | `float` | `0.0` | Per-sample Gaussian noise std on absolute yaw (rad) |
| `yaw_drift_std_rad` | `float` | `0.0` | OU yaw-drift steady-state std (rad). Set to `0` to disable. |
| `yaw_drift_time_constant_s` | `float` | `300.0` | OU mean-reversion time constant for yaw drift (s) |
