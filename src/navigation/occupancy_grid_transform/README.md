# occupancy_grid_transform

This package consumes the occupancy grid provided by CV, adds a 1-cell occupied border on the left, right, and far edges, applies obstacle inflation, and republishes two grids suitable for planning. The grid origin is transformed from the incoming frame to the configured output frame using TF2.

## Grid conventions

The occupancy grid from CV is expected to match the convention of [`nav_msgs/OccupancyGrid`](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)

- Row major
- +x is forward, +y is left of the robot
- Height = number of cells in +y direction, width = number of cells in +x direction
- info.origin is the bottom left corner of the occupancy grid in the given frame
- Indexed with (y, x) in 2d, and (y * width + x) in 1d.

## Subscribed Topics

- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Input occupancy grid in CV frame
- `mission_state` (`maverick_msgs/MissionState`) - Mission state (latched); selects which inflation parameter set is applied

## Published Topics

- `transformed_occupancy_grid` (`nav_msgs/OccupancyGrid`) - Bordered grid in the configured output frame
- `inflated_occupancy_grid` (`nav_msgs/OccupancyGrid`) - Bordered and inflated grid in the configured output frame

## Inflation

After the grid is converted and a border is added, obstacle inflation is applied. Inflation turns the border into a soft wall that discourages planning near the grid edges. For each occupied cell, surrounding cells are inflated based on their distance:

- Cells within `inflation_radius_cells` are set to fully occupied (100)
- Cells in the next `inflation_falloff_extent_cells` ring decay as `100 × decay^(dist - inflation_radius_cells)`
- Cells beyond `inflation_radius_cells + inflation_falloff_extent_cells` are unaffected

Two parameter sets are configured: `inflation_params` is used normally, and `no_mans_land_inflation_params` is used while the mission state has `in_no_mans_land` set. Since spaced out obstacles form the border of no man's land instead of the usual lane lines, the no man's land set uses a larger inflation radius to keep the robot further from obstacles and therefore within the border.
