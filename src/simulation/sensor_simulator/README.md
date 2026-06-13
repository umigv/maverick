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
See [`sensor_simulator_config.py`](sensor_simulator/sensor_simulator_config.py) for all parameters, defaults, and 
descriptions.
