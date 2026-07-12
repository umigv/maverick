# path_smoothing
Chaikin corner-cutting smoothing applied to a planned path before it is sent to the path tracker.

Chaikin iteration inserts points at 1/4 and 3/4 of every segment, rounding corners while keeping the start and end waypoints fixed. See [Chaikin's Algorithm](https://www.cs.unc.edu/~dm/UNC/COMP258/LECTURES/Chaikins-Algorithm.pdf) for details.

## Subscribed Topics
- `path` (`nav_msgs/Path`) - Raw path to smooth

## Published Topics
- `smoothed_path` (`nav_msgs/Path`) - Smoothed path
