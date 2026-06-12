# estop_driver
Reads from the serial e-stop device and atomically writes its state to `/tmp/estop_value.txt`. All other nodes read that 
file directly to determine whether the robot is estopped.

Does not publish the e-stop value to a ROS topic — this ensures the e-stop still works even if there are ROS network
issues.

## Written Files
- `/tmp/estop_value.txt` - e-stop state; `"1"` = estopped, `"0"` = safe to drive

## Config Parameters
See [`estop_driver_config.py`](estop_driver/estop_driver_config.py) for all parameters, defaults, and descriptions.
