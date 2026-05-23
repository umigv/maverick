# odrive_driver
Controls two ODrives (left and right) over USB. Converts `cmd_vel` twist commands to per-motor velocity setpoints and
publishes encoder-derived velocity with propagated covariance. Motors are zeroed if no `cmd_vel` is received within
`cmd_vel_timeout_s` or if the e-stop is active.

## Read Files
- `/tmp/estop_value.txt` - e-stop state written by `estop_driver`; commands zero velocity to both motors while estopped

## Subscribed Topics
- `cmd_vel` (`geometry_msgs/Twist`)

## Published Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - encoder derived linear/angular velocity with dynamic 
covariance

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_period_s` | `float` | `0.01` | Period (s) of the encoder publish timer |
| `cmd_vel_timeout_s` | `float` | `0.5` | Max age of a `cmd_vel` command before motors are zeroed (s) |
| `timestamp_delay_s` | `float` | `0.0` | Subtracted from the publish timestamp to compensate read and processing latency (s) |
| `frame_id` | `str` | `"base_link"` | TF frame ID attached to the published twist header |
| `estop_file_path` | `Path` | `/tmp/estop_value.txt` | Path to the e-stop flag file |

### ODrive Units
Each ODrive is identified by its USB serial number and a polarity correction. `left_odrive` and `right_odrive` each take the same parameters:

| Parameter | Type | Description |
|---|---|---|
| `serial` | `str` | USB serial number of the ODrive unit (find with `odrivetool`) |
| `polarity` | `int` | Sign correction mapping motor-native direction to robot-forward (`1` or `-1`) |

### Geometry
| Parameter | Type | Default | Description |
|---|---|---|---|
| `track_width_m` | `float` | `0.764` | Distance between left and right wheel contact points (m) |
| `wheel_diameter_m` | `float` | `0.18423` | Diameter of each drive wheel (m) |
| `gear_ratio` | `float` | `170/9 ≈ 18.89` | Motor-to-wheel gear ratio (motor revolutions per wheel revolution) |

### Controller
All parameters are specified in SI / robot-frame units and converted to motor-native units before being written to the ODrive.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `vel_gain` | `float` | `0.08` | Velocity controller proportional gain (A / (m/s)) |
| `vel_integrator_gain` | `float` | `0.0` | Velocity controller integrator gain (A / (m/s · s)) |
| `vel_integrator_limit` | `float` | `0.0` | Integrator output clamp (A); `0.0` disables the limit |
| `vel_limit_mps` | `float` | `3.0` | Motor velocity hard limit (m/s); trips an ODrive error if exceeded |
| `accel_limit_mps2` | `float` | `3.0` | Maximum linear acceleration via ODrive velocity ramp (m/s²) |
| `inertia` | `float` | `0.0` | Feed-forward inertia compensation (Nm / (m/s²)) |

### Covariance
Variance scales with speed to reflect increased uncertainty at higher velocities:

- `linear_variance  = linear_variance_static  + linear_variance_gain  * linear_mps²`
- `angular_variance = angular_variance_static + angular_variance_gain * (linear_mps² / track_width_m² + angular_radps²)`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_variance_static` | `float` | `1e-6` | Baseline linear velocity variance, independent of speed (m²/s²) |
| `linear_variance_gain` | `float` | `0.0004` | Speed-dependent gain on linear velocity variance (m²/s² per (m/s)²) |
| `angular_variance_static` | `float` | `1e-6` | Baseline angular velocity variance, independent of speed (rad²/s²) |
| `angular_variance_gain` | `float` | `0.0004` | Speed-dependent gain on angular velocity variance (rad²/s² per (m/s)²) |

## Motor calibration
Before running any script, make sure `odrivetool` and this node is closed first as they hold the USB connection.

Calibrate the odrive every time they turn on after a power cycle.
```sh
just calibrate-odrive
```
You should hear a beep when calibration starts. The indicator light will flash green while calibrating and be solid blue
when done. If the indicator light is red, run the script again.

If an ODrive errors (flashing red) at runtime:
```sh
just clear-odrive-errors
```
The indicator light should stop flashing red.

If errors persist or if the scripts can't detect an ODrive, troubleshoot in the following order:
1. Re-check power and the physical e-stop
2. Unplug and replug the ODrive from both the USB hub and the computer USB port
3. Run `odrivetool` and see if the official tool is able to detect them
4. Close the terminal and open a new one
5. Restart the laptop
