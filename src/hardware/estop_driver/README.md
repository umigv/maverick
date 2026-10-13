# estop_driver

Reads from the serial e-stop device and publishes its state. Starts stopped until a valid serial state is received.

Valid serial lines (`0` or `1`) reset a watchdog. Missing valid serial data for `serial_timeout_s` (default 1 s) asserts estop.

## Published Topics

- `estop` (`std_msgs/Bool`) - E-stop state; `true` = estopped, `false` = safe to drive. Latched and publishes a heartbeat every `poll_period_s` (default 0.05 s), with immediate publication on state changes.
