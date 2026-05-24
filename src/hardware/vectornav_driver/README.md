# vectornav_driver

ROS 2 driver node for VectorNav VN-100, VN-200, and VN-300 IMU/INS devices.

## Published Topics

| Topic | Type | Models |
|---|---|---|
| `vectornav/imu` | `sensor_msgs/Imu` | All |
| `vectornav/fix` | `sensor_msgs/NavSatFix` | VN-200, VN-300 |
| `vectornav/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | VN-200, VN-300 |
| `vectornav/ins_status` | `std_msgs/UInt16` | VN-200, VN-300 |
| `vectornav/gnss_status` | `std_msgs/UInt16` | VN-200, VN-300 |
| `vectornav/gnss2_status` | `std_msgs/UInt16` | VN-300 |
| `vectornav/odom` | `nav_msgs/Odometry` | VN-200, VN-300 |

`vectornav/odom` is only published when `datum` is set.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/vn300` | Serial port the sensor is connected to |
| `baud_rate` | `115200` | Serial baud rate. Valid: `9600 19200 38400 57600 115200 128000 230400 460800 921600` |
| `publish_rate` | `100` | Output rate in Hz. Must be a valid divisor of the sensor sample rate (800 Hz for VN-100/200, 400 Hz for VN-300) |
| `imu_frame_id` | `vectornav` | TF frame ID for the IMU origin |
| `ins_frame_id` | `vectornav` | TF frame ID for the INS reference point. Set to a different frame if the INS reference is offset from the IMU |
| `gnss_a_frame_id` | — | *(VN-200/300, required)* TF frame ID for GNSS antenna A |
| `gnss_b_frame_id` | — | *(VN-300, required)* TF frame ID for GNSS antenna B |
| `linear_accel_covariance` | `diag(0)` | 3×3 row-major covariance for linear acceleration (m/s²)² in FLU body frame |
| `angular_vel_covariance` | `diag(0)` | 3×3 row-major covariance for angular velocity (rad/s)² in FLU body frame |
| `datum` | — | *(VN-200/300, optional)* ENU odometry origin as `[lat, lon, alt]`. Required to publish odometry |
| `map_frame_id` | `map` | TF frame ID for the odometry map frame |

## TF Requirements

At startup the driver looks up TF transforms to configure sensor offsets on the device. All transforms must be available 
within 10 seconds or the driver will exit.

| Transform | Required when |
|---|---|
| `imu_frame_id` → `ins_frame_id` | VN-200/300, and `imu_frame_id != ins_frame_id` |
| `imu_frame_id` → `gnss_a_frame_id` | VN-200/300 |
| `imu_frame_id` → `gnss_b_frame_id` | VN-300 |

Offsets are provided to the sensor in FRD body frame; the driver converts from FLU automatically.

## Message Filtering

Messages are silently dropped when the sensor reports bad state.

**IMU** — dropped when:
- `imuErr != 0`
- `mode == ALIGNING (1)` — INS filter is initializing, attitude not yet valid

**NavSatFix / Odometry** — additionally dropped when:
- `gnssErr != 0`
- `gnssFix == 0`
- `mode == NOT_TRACKING (0)` or `mode == GNSS_LOST (3)`

**Twist** — dropped when:
- `imuErr != 0`
- `mode == NOT_TRACKING (0)` or `mode == GNSS_LOST (3)`

**Odometry** — additionally requires `datum` to be set.

## Frame Conventions

The sensor outputs data in NED (North-East-Down) / FRD (Forward-Right-Down). The driver converts all outputs to ROS-standard ENU (East-North-Up) / FLU (Forward-Left-Up) before publishing.

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
