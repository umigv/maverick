# common_description
Shared xacro macros for sensors, wheels, and physics used across robot descriptions.

## Structure
```
urdf/
├── physics/
│   └── inertia.xacro       # Box, cylinder, and sphere inertia macros
├── sensor/
│   ├── ann_mb.xacro        # u-blox ANN-MB GNSS antenna
│   ├── calian_tw2712.xacro # Calian TW2712 single-band GNSS antenna
│   ├── vn300.xacro         # VectorNav VN-300 Rugged Dual GNSS/INS
│   └── zed2i.xacro         # Stereolabs ZED 2i stereo depth camera
└── wheel/
    ├── caster.xacro         # Zero-friction caster wheel
    └── wheel.xacro          # Powered drive wheel
```

## Macro Conventions
All component macros follow a consistent pattern:
- **`name`** — prefix for all generated link and joint names
- **`parent`** — parent link to attach to, defaults to `base_link`
- **`*joint_origin`** — block parameter specifying the pose of the component's mount point relative to `parent`

The mount point (`{name}_base_link`) is always at the physically meaningful reference location (e.g. bottom center of 
housing, ARP for antennas). The primary measurement frame (`{name}_link`) is offset from there.


## Physics

### `box_inertia`
Inertial properties for a constant-density box. Params: `mass`, `length`, `width`, `height`, `*origin`.

### `cylinder_inertia`
Inertial properties for a constant-density cylinder. Params: `mass`, `radius`, `height`, `*origin`.

### `sphere_inertia`
Inertial properties for a constant-density sphere. Params: `mass`, `radius`, `*origin`.


## Wheels

### `wheel` — Powered Drive Wheel
- Shape: cylinder with continuous joint, axis along Y
- **`{name}_link`**: Center of the wheel

### `caster` — Caster Wheel
- Shape: sphere with fixed joint and zero friction
- **`{name}_link`**: Center of the sphere


## Sensors

### `ann_mb` — u-blox ANN-MB GNSS Antenna
- **Dimensions**: 82.0 × 60.0 × 22.5 mm, 173g (including cable)
- **`{name}_base_link`**: ARP (Antenna Reference Point) at bottom center of housing — use this as the mount origin
- **`{name}_link`**: L1 phase center, 8.9mm above ARP — the GPS measurement reference frame

### `calian_tw2712` — Calian TW2712 Single-Band GNSS Antenna
- **Dimensions**: 57.0mm diameter × 15.0mm height (cylindrical), 110g (excluding cable)
- **`{name}_base_link`**: ARP (Antenna Reference Point) at bottom center of housing — use this as the mount origin
- **`{name}_link`**: L1 phase center (offset from ARP not published; currently set to 0 — update if calibration data becomes available)

### `vn300` — VectorNav VN-300 Rugged Dual GNSS/INS
- **Dimensions**: 45 × 44 × 11 mm, 30g
- **`{name}_base_link`**: Bottom center of housing — use this as the mount origin
- **`{name}_link`**: Geometric center of housing (IMU measurement frame)
- Note: GNSS antennas A and B are modeled separately

### `zed2i` — Stereolabs ZED 2i Stereo Depth Camera
- **Dimensions**: 175.3 × 30.3 × 43.1 mm, 230g
- **`{name}_base_link`**: Bottom center of camera body — use this as the mount origin
- **`{name}_link`**: Geometric center of camera body
- **`{name}_optical_link`**: Optical frame (Z forward, X right, Y down per ROS convention)
