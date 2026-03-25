# maverick_description
URDF description for Maverick. Publishes TF transforms for all robot links via `robot_state_publisher`.

## Launch
```
ros2 launch urdf_launch display.launch.py urdf_package:=maverick_description urdf_package_path:=urdf/maverick.xacro
```

Opens RViz with the robot model and `joint_state_publisher_gui` for interactive joint control.

## Xacro Args
| Arg | Default | Description |
|---|---|---|
| `base_frame_id` | `base_link` | Root link name |
| `imu_name` | `imu` | IMU name prefix |
| `gnss_a_name` | `gnss_a` | GNSS antenna A name prefix |
| `gnss_b_name` | `gnss_b` | GNSS antenna B name prefix |

## Published TF Frames
| Frame | Description |
|---|---|
| `base_link` | Root frame, between drive wheels at ground level |
| `chassis_link` | Chassis box, offset from `base_link` |
| `right_wheel_link` | Right drive wheel center (continuous joint) |
| `left_wheel_link` | Left drive wheel center (continuous joint) |
| `caster_link` | Front caster wheel center |
| `{imu_name}_base_link` | VN-300 bottom center (mount reference) |
| `{imu_name}_link` | VN-300 IMU measurement frame |
| `{gnss_a_name}_base_link` | GNSS A antenna ARP (mount reference) |
| `{gnss_a_name}_link` | GNSS A L1 phase center |
| `{gnss_b_name}_base_link` | GNSS B antenna ARP (mount reference) |
| `{gnss_b_name}_link` | GNSS B L1 phase center |
