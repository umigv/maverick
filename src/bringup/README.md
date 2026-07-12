# bringup
Launch files and configuration for the navigation stack.

## Configurations

### Mode
There are three modes: `autonav`, `self_drive` and `nav_test`. They should be passed in with the `mode` flag. See the
documentation of each launch file to see what different modes do.

### Simulation
Pass `simulation:=true` to use simulated sensors instead of hardware

### Course
To configure a new course, add a subfolder under `bringup/courses/` containing:
- `gps.json` — GPS datum and waypoints
- `map.json` — simulation obstacle map

Courses can be generated using the [course creation tool](https://github.com/umigv/course_creation_tool). The `default`
course is used when no `course` argument is provided. See `bringup/courses/default/` for the expected schema. Pass the
subfolder name into launch files to select a course


## gps_origin_calculator.launch.py
Computes and records the GPS datum for a course. Run this once with the robot stationary at the start position before
an autonomous run. Collects GPS samples, writes the median lat/lon/alt into the course's `gps.json`, then shuts down
automatically.

```
ros2 launch bringup gps_origin_calculator.launch.py [course:=<course>]
```

### Parameters
- `course`: Course profile in `courses/` whose `gps.json` will be updated, default `default`

### Subscribed Topics
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix (e.g. from VectorNav INS)


## base.launch.py
Launches the base stack required for all operation modes: core, hardware/simulation, and localization.

```
ros2 launch bringup base.launch.py mode:=<mode> [simulation:=true] [course:=<course>]
```

### Parameters
- `mode`: Operation mode, passed through to hardware/simulation and localization launch files (required)
- `simulation`: Use simulation instead of hardware sensors, default `false`
- `course`: Course profile, default `default` (required for `mode:=autonav` or `simulation:=true`)


## core.launch.py
Launches core functionalities of the stack.

```
ros2 launch bringup core.launch.py
```

### Robot State Publisher
Loads `maverick_description/urdf/maverick.xacro` and publishes TF transforms for all robot links.

### Subscribed Topics
- `teleop_cmd_vel` (`geometry_msgs/TwistStamped`) - Joystick velocity
- `recovery_cmd_vel` (`geometry_msgs/TwistStamped`) - Recovery velocity
- `nav_cmd_vel` (`geometry_msgs/TwistStamped`) - Nav velocity

### Published Topics
- `robot_description` (`std_msgs/String`) - URDF robot description
- `cmd_vel` (`geometry_msgs/TwistStamped`) - Multiplexed output velocity

### Broadcasted TF Frames
See `maverick_description` for the full list of published frames.

### Velocity Multiplexing
| Priority | Topic | Source | Timeout |
|---|---|---|---|
| 3 | `teleop_cmd_vel` | Joystick | 0.5s |
| 2 | `recovery_cmd_vel` | Recovery system | 0.5s |
| 1 | `nav_cmd_vel` | Autonomy | 0.5s |

If a higher-priority source stops publishing, control falls back to the next source after 0.5s.


## hardware.launch.py
Launches hardware drivers

```
ros2 launch bringup hardware.launch.py mode:=<mode> [course:=<course>]
```

### Parameters
- `mode`: Operation mode (required)
- `course`: Course profile in `courses/` to load GPS datum from, default `default` (required for `autonav`)

### Modes
- `autonav`: estop + LED + ODrive + VectorNav + INS odometry
- `self_drive`: estop + LED + ODrive + VectorNav
- `nav_test`: estop + LED + ODrive

### Subscribed Topics
- `cmd_vel` (`geometry_msgs/TwistStamped`) - Multiplexed velocity command driven by ODrive (all modes)
- `teleop_cmd_vel` (`geometry_msgs/TwistStamped`) - Joystick velocity, used by led_driver to detect teleop activity (all modes)
- `nav_cmd_vel` (`geometry_msgs/TwistStamped`) - Nav velocity, used by led_driver to detect nav stack activity when no mission state is available (from `navigation.launch.py`; all modes)
- `mission_state` (`maverick_msgs/MissionState`) - Latched mission state used by led_driver for the state LEDs (from `navigation.launch.py`)

### Published Topics
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Encoder velocity from ODrive (all modes)
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data from VectorNav (`autonav`, `self_drive`)
- `gps/raw` (`sensor_msgs/NavSatFix`) - GPS/INS fix from VectorNav (`autonav`, `self_drive`)
- `ins_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - INS body-frame velocity (`autonav`, `self_drive`)
- `odom/global` (`nav_msgs/Odometry`) - INS odometry in map frame (`autonav` only)


## simulation.launch.py
Launches simulated sensors. Included by `base.launch.py` when `simulation:=true`.

```
ros2 launch bringup simulation.launch.py [course:=<course>]
```

### Parameters
- `course`: Course profile in `courses/` to load map and GPS datum from, default `default`

### Subscribed Topics
- `cmd_vel` (`geometry_msgs/TwistStamped`) - Velocity the robot is commanded to move in

### Published Topics
- `imu/raw` (`sensor_msgs/Imu`) - Simulated IMU
- `gps/raw` (`sensor_msgs/NavSatFix`) - Simulated GPS fix
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity
- `ins_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated INS body frame velocity
- `odom/global` (`nav_msgs/Odometry`) - Simulated INS odometry in `map` frame
- `odom/ground_truth` (`nav_msgs/Odometry`) - Noiseless true pose in `map` frame
(`child_frame_id = base_link_ground_truth`)
- `occupancy_grid/raw` (`nav_msgs/OccupancyGrid`) - Robot-centric occupancy grid from static obstacle map
- `occupancy_grid/ground_truth` (`nav_msgs/OccupancyGrid`) - Full static obstacle map (latched)

### Broadcasted TF Frames
- `map` → `base_link_ground_truth` - Noiseless true robot pose


## localization.launch.py
Launches localization

```
ros2 launch bringup localization.launch.py mode:=<mode> [course:=<course>]
```

### Parameters
- `mode`: Operation mode (required)
- `course`: Course profile in `courses/` to load GPS datum from, default `default` (required for `autonav`)

### Modes
`autonav`: `ekf_local` + `map_odom_publisher` + `lat_lon_converter`
- `odom` → `base_link`: EKF fusing encoder vx and IMU yaw rate
- `map` → `odom`: computed from VectorNav INS odometry (`odom/global`) and the local TF tree via `T_map_odom = T_map_base * T_odom_base⁻¹`
- `fromLL` service: converts GPS coordinates to map-frame points using the course datum

`self_drive`: `ekf_local` + identity `map` → `odom`
- `odom` → `base_link`: EKF fusing encoder vx and IMU yaw rate
- `map` → `odom`: fixed identity transform (no global correction)

`nav_test`: `enc_odom_publisher` + identity `map` → `odom`
- `odom` → `base_link`: direct encoder velocity integration, no IMU
- `map` → `odom`: fixed identity transform (no global correction)

### Subscribed Topics
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Encoder velocity (`autonav`, `self_drive`, `nav_test`)
- `imu/raw` (`sensor_msgs/Imu`) - IMU data (`autonav`, `self_drive`)
- `odom/global` (`nav_msgs/Odometry`) - VectorNav INS odometry in map frame (`autonav`)

### Published Topics
- `odom/local` (`nav_msgs/Odometry`) - Local odometry in the odom frame

### Broadcasted TF Frames
- `odom` → `base_link`
- `map` → `odom`

### Services
- `fromLL` (`robot_localization/FromLL`) - Converts GPS latitude/longitude to a map-frame point (`autonav` only)


## navigation.launch.py
Launches the navigation stack.

```
ros2 launch bringup navigation.launch.py mode:=<mode> [course:=<course>]
```

### Parameters
- `mode`: Operation mode (required)
- `course`: Course profile in `courses/` to load waypoints from, default `default` (required for `autonav`)

### Modes
All modes run occupancy grid transform + path planning + path smoothing + path tracking. `autonav` additionally runs
mission control + goal selection + recovery.

### Subscribed Topics
- `goal` (`geometry_msgs/PointStamped`) - Goal for path planning (`self_drive`, `nav_test` only)
- `occupancy_grid/raw` (`nav_msgs/OccupancyGrid`) - Raw occupancy grid from CV
- `odom/local` (`nav_msgs/Odometry`) - Odometry from localization
- `odom/global` (`nav_msgs/Odometry`) - INS odometry in the map frame, used by autonav_mission_control for waypoint tracking (`autonav` only)

### Published Topics
- `mission_state` (`maverick_msgs/MissionState`) - Latched mission state from autonav_mission_control (`autonav` only)
- `nav_cmd_vel` (`geometry_msgs/TwistStamped`) - Velocity command consumed by twist_mux
- `recovery_cmd_vel` (`geometry_msgs/TwistStamped`) - Recovery velocity command consumed by twist_mux (`autonav` only)

### Services
- `request_recovery` (`std_srvs/Trigger`) - Sets `in_recovery` in the mission state (`autonav` only)
- `recovery_complete` (`std_srvs/Trigger`) - Clears `in_recovery` in the mission state (`autonav` only)

### Service Clients
- `fromLL` (`robot_localization/FromLL`) - Converts GPS coordinates to map-frame points; called at startup by autonav_mission_control (`autonav` only)


## teleop.launch.py
Launches joystick teleoperation

```
ros2 launch bringup teleop.launch.py controller:=<controller>
```

### Parameters
- `controller`: Controller profile (`xbox`, `xbox_wireless`, `ps4`, or `ps4_wireless`   ), required

### Controller Mappings
For all controllers:
- Left joystick - linear motion
- Right joystick - turning
- Right shoulder button (RB / R1) - enable
- Left shoulder button (LB / L1) - turbo

### Published Topics
- `joy` (`sensor_msgs/Joy`) - Raw joystick input
- `teleop_cmd_vel` (`geometry_msgs/TwistStamped`) - Joystick velocity command


## visualization.launch.py
Launches visualization tools for Foxglove Studio.

```
ros2 launch bringup visualization.launch.py
```

### Foxglove Bridge
Starts Foxglove bridge on `ws://localhost:8765`.

### Occupancy Grid Voxel Visualization
Subscribes to `occupancy_grid` and republishes it as a `foxglove_msgs/VoxelGrid` on `occupancy_grid/voxels` for 3D voxel
visualization in Foxglove Studio.

### Subscribed Topics
- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Occupancy grid input

### Published Topics
- `occupancy_grid/voxels` (`foxglove_msgs/VoxelGrid`) - Voxel representation of the occupancy grid
