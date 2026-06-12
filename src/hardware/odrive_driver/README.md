# odrive_driver
Controls two ODrives (left and right) over USB. Converts `cmd_vel` twist commands to per-motor velocity setpoints and
publishes encoder-derived velocity with propagated covariance. Motors are zeroed if no `cmd_vel` is received within
`cmd_vel_timeout_s` or if the e-stop is active.

## Read Files
- `estop_file_path` (wired by `hardware.launch.py` from `launch_utils.ESTOP_FILE_PATH`) - e-stop state written by `estop_driver`; commands zero velocity to both motors while estopped

## Subscribed Topics
- `cmd_vel` (`geometry_msgs/Twist`)

## Published Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - encoder derived linear/angular velocity with dynamic 
covariance

## Config Parameters
See [`odrive_driver_config.py`](odrive_driver/odrive_driver_config.py) for all parameters, defaults, and descriptions.

### ODrive Units
Each ODrive is identified by its USB serial number and a polarity correction. `left_odrive` and `right_odrive` each take
the same parameters:

### Controller
All parameters are specified in SI / robot-frame units and converted to motor-native units before being written to the ODrive.

### Covariance
Variance scales with speed to reflect increased uncertainty at higher velocities:
- `linear_variance  = linear_variance_static  + linear_variance_gain  * linear_mps²`
- `angular_variance = angular_variance_static + angular_variance_gain * (linear_mps² / track_width_m² + angular_radps²)`

## Scripts
Before running any script, make sure `odrivetool` and this node is closed first as they hold the USB connection.

### `scripts/calibrate_odrive.py`
Calibrate the odrive every time they turn on after a power cycle.
```bash
just calibrate-odrive
```
You should hear a beep when calibration starts. The indicator light will flash green while calibrating and be solid blue
when done. If the indicator light is red, run the script again.

### `scripts/clear_odrive_errors.py`
Run if an ODrive errors (flashing red) at runtime:
```bash
just clear-odrive-errors
```
The indicator light should stop flashing red.

If errors persist or if the scripts can't detect an ODrive, troubleshoot in the following order:
1. Re-check power and the physical e-stop
2. Unplug and replug the ODrive from both the USB hub and the computer USB port
3. Run `odrivetool` and see if the official tool is able to detect them
4. Close the terminal and open a new one
5. Restart the laptop
