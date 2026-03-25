# marvin_description
URDF description for Marvin. Publishes TF transforms for all robot links via `robot_state_publisher`.

## Launch
```
ros2 launch urdf_launch display.launch.py urdf_package:=marvin_description urdf_package_path:=urdf/marvin.xacro
```

Opens RViz with the robot model and `joint_state_publisher_gui` for interactive joint control.

## Xacro Args
Frame names default to standard names.

| Arg | Default | Description |
|---|---|---|
| `base_frame_id` | `base_link` | Root link name |
| `imu_name` | `imu` | IMU name prefix — produces `{imu_name}_link` |
| `gps_name` | `gps` | GPS antenna name prefix — produces `{gps_name}_link` |
| `zed_name` | `zed` | Camera name prefix — produces `{zed_name}_link` |

## Published TF Frames
| Frame | Description |
|---|---|
| `base_link` | Robot ground frame, between drive wheels |
| `chassis_link` | Main chassis body |
| `right_wheel_link` | Right drive wheel |
| `left_wheel_link` | Left drive wheel |
| `caster_link` | Front caster wheel |
| `{zed_name}_base_link` | Camera mount point (bottom center of ZED 2i) |
| `{zed_name}_link` | Camera body center |
| `{zed_name}_optical_link` | Camera optical frame (Z forward, X right, Y down) |
| `{imu_name}_base_link` | VN-300 mount point (bottom center of housing) |
| `{imu_name}_link` | VN-300 body center (IMU measurement frame) |
| `{gps_name}_base_link` | ANN-MB ARP (Antenna Reference Point) |
| `{gps_name}_link` | ANN-MB L1 phase center (GPS measurement frame) |
