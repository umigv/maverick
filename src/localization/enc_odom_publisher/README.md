# enc_odom_publisher
Integrates wheel encoder velocity to produce odometry using unicycle kinematics. Publishes an `Odometry` message and
broadcasts the `odom → base_link` TF transform.

Intended as a drop-in replacement for `ekf_local` when IMU-corrected local odometry is not needed or desired. The global
EKF (`ekf_global`) can still fuse this with GPS and absolute IMU for a map-frame estimate.

## Subscribed Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - encoder velocity

## Published Topics
- `odom` (`nav_msgs/Odometry`) - integrated pose and velocity

## TF Broadcasts
- `odom → base_link` - derived from `odom_frame_id` and `base_frame_id` config

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `pose_x_variance_m2` | `0.01` | Pose covariance for x (m²) |
| `pose_y_variance_m2` | `0.01` | Pose covariance for y (m²) |
| `pose_yaw_variance_rad2` | `0.01` | Pose covariance for yaw (rad²) |
| `max_dt_s` | `1.0` | Maximum allowed dt (s) between encoder messages; updates exceeding this are dropped |
| `odom_frame_id` | `odom` | Parent frame for odometry and TF |
| `base_frame_id` | `base_link` | Child frame for odometry and TF |

Twist covariance is propagated directly from the encoder driver message. Pose covariance is fixed diagonal. Tune these
based on your encoder's expected drift characteristics.
