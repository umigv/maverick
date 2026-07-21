# common_description

Shared xacro macros for sensors, wheels, and physics used across robot descriptions.

## Macro Conventions

All component macros follow a consistent pattern:

- **`name`** - Prefix for all generated link and joint names
- **`parent`** - Parent link to attach to, defaults to `base_link`
- **`*joint_origin`** - Block parameter specifying the pose of the component's mount point relative to `parent`

The mount point (`{name}_base_link`) is always at the physically meaningful reference location (e.g. bottom center of housing, ARP for antennas). The primary measurement frame (`{name}_link`) is offset from there.

______________________________________________________________________

## Physics

### `box_inertia`

Inertial properties for a constant-density box. Params: `mass`, `length`, `width`, `height`, `*origin`.

### `cylinder_inertia`

Inertial properties for a constant-density cylinder. Params: `mass`, `radius`, `height`, `*origin`.

### `sphere_inertia`

Inertial properties for a constant-density sphere. Params: `mass`, `radius`, `*origin`.

______________________________________________________________________

## Wheels

### `wheel` - Powered Drive Wheel

- Shape: cylinder with continuous joint, axis along Y
- **`{name}_link`**: Center of the wheel

### `caster` - Caster Wheel

- Shape: sphere with fixed joint and zero friction
- **`{name}_link`**: Center of the sphere

______________________________________________________________________

## Sensors

### `ann_mb` - u-blox ANN-MB GNSS Antenna

- **Dimensions**: 82.0 x 60.0 x 22.5 mm, 173g (including cable)
- **`{name}_base_link`**: ARP (Antenna Reference Point) at bottom center of housing - use this as the mount origin
- **`{name}_link`**: L1 phase center, 8.9mm above ARP - the GPS measurement reference frame

### `calian_tw2712` - Calian TW2712 Single-Band GNSS Antenna

- **Dimensions**: 57.0mm diameter x 15.0mm height (cylindrical), 110g (excluding cable)
- **`{name}_base_link`**: ARP (Antenna Reference Point) at bottom center of housing - use this as the mount origin
- **`{name}_link`**: L1 phase center (offset from ARP not published; currently set to 0 - update if calibration data becomes available)

### `vn300` - VectorNav VN-300 Rugged Dual GNSS/INS

- **Dimensions**: 45 x 44 x 11 mm, 30g
- **`{name}_base_link`**: Bottom center of housing - use this as the mount origin
- **`{name}_link`**: Geometric center of housing (IMU measurement frame)
- Note: GNSS antennas A and B are modeled separately

### `zed2i` - Stereolabs ZED 2i Stereo Depth Camera

- **Dimensions**: 175.3 x 30.3 x 43.1 mm, 230g
- **`{name}_base_link`**: Bottom center of camera body - use this as the mount origin
- **`{name}_link`**: Geometric center of camera body
- **`{name}_optical_link`**: Optical frame (Z forward, X right, Y down per ROS convention)
