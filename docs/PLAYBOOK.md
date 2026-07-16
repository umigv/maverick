# Playbook

How to operate the physical robot on a test or competition day. For launch commands and what each mode does, see the [README](../README.md) and [bringup/README.md](../src/bringup/README.md).

Everything runs directly on a laptop mounted on the robot - there is no remote connection to set up.

## Field day sequence

1. [Mech prep](#mech-prep)
2. [Power on](#power-system): motor power switch on, breaker engaged, sensor power bank on
3. Turn on the [remote e-stop](#remote-e-stop)
4. [Calibrate the motors](#odrive)
5. Wait for GPS startup: run `just vectornav-monitor` to watch INS and GNSS status live (see [vectornav_driver/README.md](../src/hardware/vectornav_driver/README.md))
6. Record the GPS datum: with the robot stationary at the start position, run `ros2 launch bringup gps_origin_calculator.launch.py course:=<course>` - it writes the datum into the course's `gps.json` and shuts down automatically (see [Course setup](#course-setup))
7. Launch the stack (see [README](../README.md) for commands and modes)
8. Record rosbags for every run (see [Post-run](#post-run) for naming and upload)

## Course setup

Courses live in `src/bringup/courses/` - one subfolder per course, selected with the `course:=` launch argument. To set up at a new site, create a course folder with the [course creation tool](https://github.com/umigv/course_creation_tool); see [bringup/README.md](../src/bringup/README.md) for the schema.

On real runs only `gps.json` (GPS datum and waypoints) is used, so that is what needs to be filled out. `map.json` is only the simulation obstacle map. The datum in `gps.json` is recorded on-site with the GPS origin calculator (step 6 of the [field day sequence](#field-day-sequence)).

## Mech prep

- Check for loose screws. If any are loose, add a lock washer.
- Lube the gearbox with dry lube. Do it outside since the smell stays.
- Wipe the wheels down with isopropyl alcohol before running - it impacts wheel friction.
- Make sure the wheel shield isn't scraping the wheel. If the wheel is caving, check whether the gearbox plate is bent.
- Run the robot in the same configuration as it runs at comp, including wiring and weight. After any physical change (cable management, remounting), test again before it counts: at comp the USB hub cable got moved right beside the GPS cable the day before and was never re-tested.
- Before heading outside, make sure what you want to test is actually ready. Setup eats a ton of time, and warm testing weather is precious.

## Post-run

- Turn off the motor power switch so the LiPO doesn't drain.
- Put both batteries on charge. The Anker lasts longer and charges faster than the Jackery, so prioritize the Anker, but make sure the other is charging while you use one.
- Upload rosbags to Dropbox at the end of the day. Name them so people can tell what they are. Delete useless rosbags, and don't commit them to the repo. (TODO: link the Dropbox folder)
- TODO: note what broke or acted weird while it's fresh - where do issues get logged?

## Troubleshooting

When things go wrong: suspect hardware more than you think (consider what is different vs simulation), and when investigating a regression, go through everything that changed since it last worked.

**ODrive says the e-stop is engaged but it isn't.** Check the wiring across the system, starting with the power connections - something likely came loose.

**ODrive hits the current limit.** Lube the gearbox and reduce kp on the ODrive - both lower the torque needed to drive the gearbox, which reduces current.

**Odometry is off / robot goes crazy on a simple turn at high speed.** Suspect wheel slip - it's hard to diagnose and happens when the robot moves, accelerates, or turns too fast. Wipe the wheels with isopropyl and lower the speed (see [Mech prep](#mech-prep)). Consider the floor's coefficient of friction when testing (asphalt > cement > marble).

**Robot doesn't follow paths precisely.** Likely inertia - the controllers may need retuning. Note that weight changes affect path tracking tuning, and gear ratio or wheel diameter changes affect odometry, so physical changes to the platform mean software retuning.

**Robot drives out of bounds in real runs but behaved in sim.** The simulation occupancy grid has no unknown cells - real runs do. Don't trust sim behavior around unknowns.

**GPS fix is bad or satellite count drops.** Run `just vectornav-monitor` to see the decoded INS and GNSS status. The VN300 antenna is sensitive to USB 3.0 EMI - this is an ongoing problem to be fixed. Make sure nothing running USB 3.0 (ZED camera, USB hub) is close to the antenna cables; move them physically as far apart as possible. Elevating the receivers also improves signal.

**Motors lost calibration.** Calibration persists across e-stop, but is lost if you unplug USB and turn off the main power. Recalibrate (see [ODrive](#odrive)).

## Power system

- There are two sources of power: the LiPO batteries for the motors, and the Anker / Jackery power bank for the laptop (which powers all the sensors).
- The motor power system has a breaker, an e-stop, and a power switch. The power switch needs to be on and the breaker engaged.
- The LiPO should generally never run out of battery. (TODO: state why - over-discharge damage? - and what voltage to stop at)
- TODO: LiPO power monitoring instructions
- TODO: LiPO charging and storage safety rules
- Everything on the robot, especially wiring, should be labeled such that when facing forward, the left (port) side is red and the right (starboard) side is green.

## Remote e-stop

- The remote e-stop is connected to a power bank. The cable has a power switch on the back.
- To turn on: make sure the power switch is on, then turn on the power bank.
- To turn off: turn off the power switch on the back.

## ODrive

- Calibrate the motors before running. You don't need to recalibrate unless you unplug USB and turn off the main power (e-stop is fine).
- Configuration is done through the [web GUI](https://gui.odriverobotics.com/#/dashboard).
- [API docs](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html)

## VN300

- The antenna cables should point in the same direction when mounted. (TODO: photo, and what goes wrong otherwise)
- Antenna mounting hardware: plastic screw from the electrical box, plus an 8-32 borrowed from MTBR. (TODO: rewrite this so someone else can actually do it - photo)
- Use the Milwaukee folding ruler to measure the offsets between the sensors. (TODO: where do the measured offsets go?)
- Documentation PDFs are on Dropbox. (TODO: link)
- VectorNav Control Center doesn't run in the virtual machine - use the team laptop for it.
- Support contact: support@vectornav.com or +1 (512) 772-3615.

## Software practices

- Use light mode when it's bright outside - easier to see.
- Use a test branch for test-day code changes, then clean it up after. Don't commit to main. Do it daily so the number of code changes doesn't explode.

## Packing list

- LiPO batteries (charged) and their charger
- Anker and Jackery power banks (charged) and their chargers
- Laptop and laptop charger
- Remote e-stop and its power bank
- Game controller for teleop (xbox or ps4)
- Isopropyl alcohol (wheel wipe)
- Dry lube (gearbox)
- Lock washers and basic tools (screwdrivers, hex keys)
- Milwaukee folding ruler (sensor offset measurements)
- TODO: spares worth carrying - fuses, connectors, zip ties?
