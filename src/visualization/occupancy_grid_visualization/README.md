# occupancy_grid_visualization

Converts a `nav_msgs/OccupancyGrid` into a `foxglove_msgs/VoxelGrid` for 3D voxel visualization in Foxglove Studio.

This node preserves the input grid's occupancy values, frame, origin, and XY resolution. It does not create a map or detect obstacles. Each cell becomes one voxel in a single layer.

## Running

```bash
ros2 run occupancy_grid_visualization foxglove_voxel
```

The `voxel_height` parameter controls the layer thickness in meters (default `0.1`; must be positive and finite):

```bash
ros2 run occupancy_grid_visualization foxglove_voxel --ros-args -p voxel_height:=0.05
```

Remap `occupancy_grid` to the desired map topic, such as `/occupancy_grid/inflated`, when needed. For a flat map in Foxglove, enable the original occupancy grid topic directly. Voxel boxes may cover path markers below their top surface; use Foxglove's Draw behind setting or display the flat map when comparing paths.

## Subscribed Topics

- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Occupancy grid to visualize

## Published Topics

- `occupancy_grid/voxels` (`foxglove_msgs/VoxelGrid`) - Voxel representation of the occupancy grid
