# odrive_driver
Controls two ODrives (left and right) over USB. Converts `cmd_vel` twist commands to per-motor velocity setpoints and
publishes encoder-derived velocity with propagated covariance. Motors are zeroed if no `cmd_vel` is received within
`cmd_vel_timeout_s` or if the e-stop is active.

## Read Files
- `estop_file_path` - e-stop state written by `estop_driver`; commands zero velocity to both motors while estopped

## Subscribed Topics
- `cmd_vel` (`geometry_msgs/TwistStamped`)

## Published Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - encoder derived linear/angular velocity with dynamic 
covariance
- `odrive_driver/debug` (`std_msgs/Float32MultiArray`) - per-motor Iq and velocity signals, only published when `debug`
is true: `[l_iq_sp, l_iq_meas, l_vel_sp, l_vel_est, r_iq_sp, r_iq_meas, r_vel_sp, r_vel_est]`. Iq in A, velocity in rps

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

### `scripts/plot_motor_signals.py`
Live-plots left/right Iq setpoint vs measured (A) and velocity setpoint vs estimate (rps) from the `odrive_driver/debug`
topic. Useful for tuning the velocity controller and diagnosing motor behavior. Requires `odrive_driver` to be running 
with `debug: true` in its config.
```bash
just plot-odrive                              # 500-sample window, 10 Hz redraw
just plot-odrive --window 1000 --frame-rate 15
```
