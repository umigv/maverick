# vectornav_driver
ROS 2 driver node for the VectorNav VN-300 IMU/INS. Publishes IMU, GNSS fix, body velocity, odometry, and status topics.
At startup it reads TF transforms to configure sensor offsets on the device, then streams binary output packets at the
configured rate.

## Published Topics
- `vectornav/imu` (`sensor_msgs/Imu`) - IMU data in FLU body frame
- `vectornav/fix` (`sensor_msgs/NavSatFix`) - GNSS position fix
- `vectornav/velocity` (`geometry_msgs/TwistWithCovarianceStamped`) - body-frame velocity
- `vectornav/odom` (`nav_msgs/Odometry`) - ENU odometry relative to datum (only published when `datum` is set)
- `vectornav/ins_status` (`std_msgs/UInt16`) - raw INS status bitfield
- `vectornav/gnss_status` (`std_msgs/UInt16`) - raw GNSS antenna A status bitfield
- `vectornav/gnss2_status` (`std_msgs/UInt16`) - raw GNSS antenna B status bitfield
- `vectornav/yaw_uncertainty` (`std_msgs/Float32`) - yaw uncertainty in degrees, published unconditionally regardless of
INS mode. Useful for monitoring heading convergence during startup
- `vectornav/gnss_compass_signal_health` (`std_msgs/Float32MultiArray`) - GNSS compass signal health: `[numSatsPvtA, 
numSatsRtkA, highestCn0A, numSatsPvtB, numSatsRtkB, highestCn0B, numComSatsPvt, numComSatsRtk]`
- `vectornav/gnss_compass_startup_status` (`std_msgs/Float32MultiArray`) - GNSS compass startup status: 
`[percentComplete, currentHeading]`

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `port` | `str` | `/dev/vn300` | Serial port the sensor is connected to |
| `baud_rate` | `int` | `115200` | Serial baud rate. Valid: `9600 19200 38400 57600 115200 128000 230400 460800 921600` |
| `measurement_publish_period_s` | `float` | `0.01` | Measurement publish period (s). Must correspond to an integer divisor of the 400 Hz sensor sample rate |
| `status_register_poll_period_s` | `float` | `1.0` | Period (s) for polling the GNSS compass signal health and startup status registers |
| `imu_frame_id` | `str` | `vectornav` | TF frame ID for the IMU origin |
| `ins_frame_id` | `str` | `vectornav` | TF frame ID for the INS reference point |
| `gnss_a_frame_id` | `str` | — | TF frame ID for GNSS antenna A (required) |
| `gnss_b_frame_id` | `str` | — | TF frame ID for GNSS antenna B (required) |
| `linear_accel_covariance` | `float[9]` | `diag(0)` | 3×3 row-major covariance for linear acceleration (m/s²)² in FLU body frame |
| `angular_vel_covariance` | `float[9]` | `diag(0)` | 3×3 row-major covariance for angular velocity (rad/s)² in FLU body frame |
| `datum` | `float[3]` | — | ENU odometry origin as `[lat, lon, alt]`. Required to publish odometry |
| `map_frame_id` | `str` | `map` | TF frame ID for the odometry map frame |
| `require_attitude` | `bool` | `true` | If true, drop IMU messages until the INS filter has valid attitude |

## TF Requirements
At startup the driver waits up to 10 seconds for TF transforms to configure sensor offsets on the device. All transforms
must be available or the driver will exit.

| Transform | Description |
|---|---|
| `imu_frame_id` → `ins_frame_id` | INS reference point offset from IMU origin (used when frames differ) |
| `imu_frame_id` → `gnss_a_frame_id` | GNSS antenna A position offset from IMU origin |
| `imu_frame_id` → `gnss_b_frame_id` | GNSS antenna B position offset from IMU origin |

Offsets are written to the sensor in FRD body frame; the driver converts from FLU automatically. If 
`imu_frame_id` → `ins_frame_id` has changed since last boot, the sensor is reset so the new offset takes effect as
InsRefOffset is a static register.

## Message Filtering
Messages are silently dropped when the sensor reports bad state.

**IMU** — dropped when:
- `ins_status` or `angular_vel` unavailable
- `imuErr != 0`
- `require_attitude = true` and attitude solution not ready (`mode == ALIGNING`, or `quat`/`accel`/`yprU` unavailable)

**NavSatFix / Odometry** — dropped when:
- `gnssErr != 0`
- `gnssFix == 0`
- `mode == NOT_TRACKING (0)` or `mode == GNSS_LOST (3)`

**Twist** — dropped when:
- `imuErr != 0`
- `mode == NOT_TRACKING (0)` or `mode == GNSS_LOST (3)`

## Frame Conventions
The sensor outputs data in NED (North-East-Down) / FRD (Forward-Right-Down). The driver converts all outputs to 
ROS-standard ENU (East-North-Up) / FLU (Forward-Left-Up) before publishing.

- **Orientation** — rotated via `q_ned_to_enu * q_in * q_frd_to_flu`
- **Angular velocity, linear acceleration, body velocity** — FRD→FLU by negating Y and Z axes

## Scripts

### `scripts/euler_monitor.py`
Prints roll, pitch, and yaw in degrees from an IMU topic.
```bash
just euler-monitor             # uses vectornav/imu by default
just euler-monitor other/imu   # custom topic
```

### `scripts/vectornav_monitor.py`
Live terminal display of INS and GNSS status, decoding the `ins_status`, `gnss_status`, and `gnss2_status` bitfields.
Useful for monitoring startup progress and diagnosing fix issues.
```bash
just vectornav-monitor
```
