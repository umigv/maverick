# occupancy_grid_visualization

Converts a `nav_msgs/OccupancyGrid` into a `foxglove_msgs/VoxelGrid` for 3D voxel visualization in Foxglove Studio.

This node preserves the input grid's occupancy values, frame, XY origin, orientation, and XY resolution. It does not create a map or detect obstacles. Each cell becomes one voxel in a single layer, raised by a configurable offset.

## Running

```bash
ros2 run occupancy_grid_visualization foxglove_voxel
```

With no arguments, the node reads `/occupancy_grid/raw`, publishes `/occupancy_grid/voxels`, and creates a layer 0.5 m tall raised 0.1 m above the input origin to reduce overlap with the flat ground-truth map.

The `voxel_height` parameter controls the layer thickness in meters (default `0.5`; must be positive and finite). The finite `z_offset` parameter controls the vertical offset in the input frame (default `0.1`):

```bash
ros2 run occupancy_grid_visualization foxglove_voxel --ros-args -p voxel_height:=0.05
```

Select another input with `--ros-args -p input_topic:=/occupancy_grid/inflated` when needed. The vertical offset reduces coplanar flickering; both maps still cover the same XY area. For a flat map in Foxglove, enable the original occupancy grid topic directly. Voxel boxes may cover path markers below their top surface; use Foxglove's Draw behind setting or display the flat map when comparing paths.

## Subscribed Topics

- `occupancy_grid/raw` (`nav_msgs/OccupancyGrid`) - Default occupancy grid to visualize; configurable with `input_topic`

## Published Topics

- `occupancy_grid/voxels` (`foxglove_msgs/VoxelGrid`) - Voxel representation of the occupancy grid
