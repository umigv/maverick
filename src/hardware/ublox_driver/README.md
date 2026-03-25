# ublox_driver
This package reads GNSS data in UBX protocol from our **u-blox ZED-F9P** GPS over USB and publishes 
`sensor_msgs/NavSatFix` messages for use with localization.

## Behavior / Filtering
We only publish message UBX `NAV-PVT`/`NAV2-PVT` that indicates a valid position solution:
- `fixType` is not `NO_FIX (0)` and not `TIME_ONLY (5)`
- `gnssFixOk == 1`
- `invalidLlh == 0`

Timestamps are taken from GNSS time when valid (`validDate && validTime && fullyResolved`), otherwise we fall back to 
ROS2 node time.

Covariance is filled using u-blox accuracy fields:
- `hAcc` / `vAcc` (mm → meters → meters²), published as diagonal covariance.

## Published Topics
- `ublox/gps` (`sensor_msgs/NavSatFix`) - processed GPS fix messages

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ublox` | Serial port device path for the GPS receiver |
| `baud_rate` | `460800` | Serial baud rate for communication |
| `poll_period_s` | `0.1` | Period in seconds between GPS polls |
| `ublox_frame_id` | `gps_link` | TF frame ID used in published GPS messages |

## GPS Configurations
> Note: Message “rate” in `CFG-MSG` is **messages per navigation cycle**.  
> With `CFG-RATE` set to **10 Hz**, a message rate of:
> - `1` → 10 Hz
> - `10` → 1 Hz

The GPS must be configured with the following on [u-center](https://www.u-blox.com/en/product/u-center):
- UBX-CFG-PRT
   - Target: 3 - USB
   - Protocol in: 0 - UBX
   - Protocol out: 0 - UBX
- UBX-CFG-MSG check USB and set the given value:
   - 01-07-NAV-PVT -> 1
   - 01-03-NAV-STATUS (debug) -> 10
   - 01-35-NAV-SAT (debug) -> 10
- UBX-CFG-MSG uncheck USB:
   - F0-00 NMEA GxGGA
   - F0-04 NMEA GxRMC
   - G0-02 NMEA GxGSA
   - G0-03 NMEA GxGSV
- UBX-CFG-RATE
   - Time Source: 0 - UTC Time
   - Measurement Period: 100ms
   - Navigation Rate: 1 cyc
- UBX-CFG-CFG send with following settings so config is saved
  - Devices: 0 - BBR & 1 - FLASH
  - Save current configuration
