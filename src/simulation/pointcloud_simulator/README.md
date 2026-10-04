# pointcloud_simulator

Simulates a 3D point cloud sensor, such as a lidar or depth camera, from a static obstacle map, standing in for real 3D sensors in simulation. Each node instance emulates one device. The mount pose and field of view are configurable, so several instances can run side by side with different offsets and FOVs. In [simulation.launch.py](../../bringup/launch/simulation.launch.py), each top-level key in [`config/simulation/pointcloud_simulator.yaml`](../../bringup/config/simulation/pointcloud_simulator.yaml) starts its own instance.

## World Model

- The map file is the same JSON course map used by [occupancy_grid_simulator](../occupancy_grid_simulator/README.md#map-file-format).
- Each `obstacles` cell is extruded into a box from `z = 0` to `z = obstacle_height_m`.
- The ground is an infinite plane at `z = 0` and returns points like any other surface.
- `lane_lines` are ignored, since they're flat on the ground and indistinguishable from it in XYZ.

## Sensor Model

- Rays are laid out on an evenly spaced azimuth × elevation grid, centered on the sensor's +x axis and spanning `horizontal_fov_rad` × `vertical_fov_rad`. A 360° horizontal FOV drops the duplicate ray at ±180°.
- Each tick, the sensor pose in the world is the ground-truth robot pose composed with the mount pose (`offset_*`, `roll_rad`, `pitch_rad`, `yaw_rad`).
- Each ray returns the first surface it hits, either the ground or an obstacle box. Obstacles are found exactly, with a vectorized 2D grid traversal (Amanatides–Woo DDA) over all rays at once. This means obstacles occlude everything behind them.
- Returns outside `[min_range_m, max_range_m]` are dropped. Gaussian noise with std `range_noise_std_m` is added along each remaining ray.
- Points are published in `sensor_frame_id`.

## Subscribed Topics

- `odom` (`nav_msgs/Odometry`) - Ground-truth robot pose in the map frame

## Published Topics

- `pointcloud` (`sensor_msgs/PointCloud2`) - XYZ point cloud stamped in `sensor_frame_id`, published periodically

## Read Files

- `map_file_path` - Course map JSON, read once on startup

## TF Broadcasts

- `base_frame_id` -> `sensor_frame_id` - Static mount pose built from the configured offset and rotation
