# occupancy_grid_visualization
Converts a `nav_msgs/OccupancyGrid` into a `foxglove_msgs/VoxelGrid` for 3D voxel visualization in Foxglove Studio.

## Subscribed Topics
- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Occupancy grid to visualize

## Published Topics
- `occupancy_grid/voxels` (`foxglove_msgs/VoxelGrid`) - Voxel representation of the occupancy grid
