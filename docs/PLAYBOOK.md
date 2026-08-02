# Playbook

How to operate the physical robot on a test or competition day. Everything runs directly on a laptop mounted on the robot.

## Field day sequence

1. [Platform preparation](#platform-preparation).
2. [Power on](#power-system-and-wiring): motor power switch on, breaker engaged, laptop power bank on.
3. Turn on the [remote e-stop](#remote-e-stop).
4. [Calibrate the motors](#odrive).
5. Launch the base stack (see [README](../README.md#starting-the-robot) for commands and modes). The hardware drivers must be running for the next two steps.
6. Wait for GPS startup: run `just vectornav-monitor` to watch INS and GNSS status live (see [vectornav_driver/README.md](../src/hardware/vectornav_driver/README.md)).
7. [Record the GPS datum](#course-setup) with the robot stationary at the start position, then restart the base stack. The datum is read at launch and dictates the origin of global odometry.
8. Launch teleop and/or navigation (see [README](../README.md#starting-the-robot)).
9. Record rosbags as needed with `ros2 bag record --all` (see [Post-run](#post-run) for naming and upload).

## What to bring

- LiFePO4 batteries
- Laptop and chargers
- Power banks and chargers
- Remote e-stop
- Chair
- Isopropyl alcohol and cloth
- A wifi source (e.g. hotspot)
- PS4 controller
- Test obstacles
- Tools for quick repairs

For comp additionally:

- A prepped replacement for every key part on the robot
- Aluminum extrusion, fastener joints, and power tools like table saw and drill for rapid prototyping on-site
- Umbrella for shade for the laptop screen
- Sunscreen

> [!WARNING]
> Comp days are long hours in direct sun!

## Course setup

Courses live in `src/bringup/courses/` - one subfolder per course, selected with the `course:=` launch argument. Only `gps.json` (GPS datum and waypoints) is used on real runs; see [bringup/README.md](../src/bringup/README.md) for the full schema.

At a new site:

1. Create the course folder `src/bringup/courses/<course>/`.
2. Fill in the waypoints. Competition waypoints are handed out in packed DMS format: store them verbatim in `waypoints/dms/` and run `just convert-waypoints` to generate the decimal-degree versions that go into `gps.json` (see [waypoints/README.md](../waypoints/README.md) for the workflow).
3. Record the datum: with `base.launch.py` running (the calculator reads the GPS topic) and the robot stationary at the start position, run `ros2 launch bringup gps_origin_calculator.launch.py course:=<course>`. It writes the datum into the course's `gps.json` and shuts down automatically.
4. (Re)launch the stack with `course:=<course>`. The datum is read at launch, so anything already running needs a restart to pick it up.

`map.json` is only the simulation obstacle map, created with the [course creation tool](https://github.com/umigv/course_creation_tool). It isn't needed for real runs.

## Platform preparation

- Check for loose screws. If any are loose, add a lock washer.
- Lube the gearbox with dry lube outside if it is not smooth.
- Wipe the wheels down with isopropyl alcohol before running to ensure consistent wheel friction.
- Make sure the wheel shield isn't scraping the wheel. If the wheel is caving, check whether the gearbox plate is bent.
- Ensure the payload (25 pounds of gym weights) is mounted on the bottom of the robot. <img src="images/payload.jpg" alt="Payload weights mounted under the deck" width="600">
- Run the robot in the same configuration as it runs at comp, including wiring, payload, and weight. After any physical change (cable management, remounting), test again before it counts.

> [!WARNING]
> At IGVC 2026 we moved the USB hub cable right beside the GPS receiver cable without retesting. This impacted signal strength and took us 3 days to diagnose.

- Before heading outside, make sure what you want to test is actually ready. Setup eats a ton of time, and warm testing weather is precious.

## Power system and wiring

- There are two sources of power: the LiFePO4 batteries for the motors, and the Anker / Jackery power bank for the laptop (which powers all the USB devices).
- The Anker lasts longer and charges faster than the Jackery, so prioritize the Anker, but make sure the other is charging while you use one.
- The motor power system has a breaker, an e-stop, and a power switch. The power switch needs to be on and the breaker engaged. <img src="images/power_system.jpg" alt="Breaker, power switch, and e-stop locations" width="600">
- The LiFePO4 should generally never run out of battery given it has 500+ hours of battery life. Download "LiFePO4 Power" ([iOS](https://apps.apple.com/us/app/lifepo4-power/id1582607413), [Android](https://play.google.com/store/apps/details?id=com.dy.leadyo&hl=en_US)) on your phone to monitor the current battery percentage.
- Everything on the robot, especially wiring, should be labeled such that when facing forward, the left side is red and the right side is green.

## Remote e-stop

- The remote e-stop is connected to a power bank. The cable has a power switch on the back. <img src="images/remote_estop.jpg" alt="Remote e-stop power bank and cable power switch" width="400">
- To turn on: make sure the power switch is on, then turn on the power bank.
- To turn off: turn off the power switch on the back.

## ODrive

- Calibrate the motors with `just calibrate-odrive` before running.
  - After you've calibrated once, you don't need to recalibrate for the rest of the session, unless you unplug USB and turn off the main power (e-stop is fine).
- Configuration is done through the [web GUI](https://gui.odriverobotics.com/#/dashboard).
- [API docs](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html)
- Support contact: info@odriverobotics.com

## VN300

To mount or remount the antennas:

1. Screw the antennas onto the ground plane using the plastic screws in the electrical box.
2. Point both antenna cables in the same direction, or the attitude reading may not converge. <img src="images/vn300_antennas.jpg" alt="Antenna cables pointing in the same direction, with the folding ruler measuring the antenna offset" width="400">
3. Measure the offsets between the sensors (base->IMU, IMU->GNSS A, GNSS A->GNSS B) with the Milwaukee folding ruler.
4. Enter the measured offsets as the sensor offset constants in [maverick_description/urdf/constants.xacro](../src/description/maverick_description/urdf/constants.xacro). At startup the [vectornav driver](../src/hardware/vectornav_driver/README.md) reads the resulting TF and writes the offsets to the sensor.

Reference:

- [Documentation PDFs](https://www.dropbox.com/scl/fo/nmoe93a92kkug9jg4yjgl/AGFn25HFFZrdIWpzx5zTuLI?rlkey=gkhee7wdr551iaipuppoqoh7o&st=2b8ens5u&dl=0)
- Support contact: support@vectornav.com or +1 (512) 772-3615

## Controller pairing

- Wired: plug the controller in over USB and launch teleop with `controller:=ps4`.
- Bluetooth: hold Share + PlayStation until the light bar flashes to enter pairing mode, then pair it in the Bluetooth settings panel. Then launch teleop with `controller:=ps4_wireless`.

## Device aliases

Hardware configs refer to devices by stable paths (`/dev/vn300`, `/dev/estop`, `/dev/led`) instead of raw `/dev/ttyUSB*` names that change between boots. On a new laptop, plug in each device and create its alias once:

```bash
just alias /dev/ttyUSB0 vn300
```

The alias is a udev rule keyed to the device's USB vendor/product/serial, so it survives replugging and reboots. `just unalias <name>` removes one.

## Code changes

- Use light mode when it's bright outside to make it easier to see.
- See [CONTRIBUTING.md](CONTRIBUTING.md#branches) for how to organize code changes made on the field.

## Post-run

- Turn off the motor power switch so the LiFePO4 doesn't drain. Put both power banks on charge.
- Upload rosbags to Dropbox at the end of the day. Name them so people can tell what they are. Delete useless rosbags, and don't commit them to the repo.
- File a GitHub issue for anything that broke or acted weird while it's fresh (see [CONTRIBUTING.md](CONTRIBUTING.md#issues)).

## Troubleshooting

When things go wrong, suspect hardware more than you think (consider what is different between the real robot and simulation).

When investigating a regression, go through everything that changed since it last worked.

Common issues:

- **ODrive says the e-stop is engaged but it isn't.** Check the wiring across the system, starting with the power connections. It's likely that a wire came loose.
- **ODrive hits the current limit.** Lube the gearbox if unlubed for a while and retune `vel_gain` downward in [odrive_driver_config.py](../src/hardware/odrive_driver/odrive_driver/odrive_driver_config.py). A lower gain draws less current but tracks commanded velocity less aggressively, so the robot may undershoot its target speed.
- **Odometry is off / robot goes crazy on a simple turn at high speed.** Suspect wheel slip. Wheel slip is hard to diagnose and happens when the robot moves, accelerates, or turns too fast. Wipe the wheels with isopropyl and lower the speed. Consider how grippy the floor is when testing (asphalt > cement > marble).
- **Robot doesn't follow paths precisely.** Likely inertia. The path following controller may need retuning. Note that weight changes affect path tracking tuning, and gear ratio or wheel diameter changes affect odometry, so physical changes to the platform mean software retuning.
- **GPS fix is bad or satellite count drops.** Run `just vectornav-monitor` to see the decoded INS and GNSS status. The VN300 antenna is sensitive to USB 3.0 EMI (TODO: fix EMI). Make sure nothing running USB 3.0 (ZED camera, USB hub) is close to the antenna cables; move them physically as far apart as possible. Elevating the receivers also improves signal.
- **ZED camera initialization fails or frames are intermittent.** Check the cable's connection to the back of the camera. The screws need to be absolutely tight.
- **ZED camera is inaccurate after self-calibration.** At every startup the ZED refines its factory stereo calibration against the scene it sees. If the scene has too little detail (too close to an object, facing empty space), the result can be worse than what it started with. Restart the camera facing a detailed scene.
