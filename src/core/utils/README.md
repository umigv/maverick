# utils
Small utilities for ROS 2 Python nodes:

## utils.config
Adds a simple config-loading utility for ROS 2.

To use, you will want to create a python dataclass where each field in the config dataclass corresponds to a ROS 2 
parameter key. The field’s default value (if provided) is used as the parameter’s default.

Mapping rules:
- Each dataclass field name corresponds to a ROS 2 parameter key.
- If the field has a default value (or default_factory), the parameter is optional and the default is used when the key
is not supplied in YAML.
- If the field has no default, the parameter is required; `load()` will raise if it is missing / unset.
- Nested dataclasses are supported and map to nested parameter dictionaries.

Supported field types:
- Primitives: `bool`, `int`, `float`, `str`, `bytes`
- Arrays: `list[bool]`, `list[int]`, `list[float]`, `list[str]`
- `pathlib.Path` (declared as a string parameter, coerced to `Path` on load)
- `typing.Literal[...]` of values sharing one supported primitive type; validated against the allowed set

For example, you can create the following config dataclass:
```py
from dataclasses import dataclass, field

@dataclass
class Weights:
    clearance: float
    heading: float = 1.0

@dataclass
class PlannerConfig:
    weights: Weights
    timeout_s: float
    max_iters: int = 10_000
```

Which would map to a ROS2 config yaml structure like this:
```yaml
planner:
  ros__parameters:
    # max_iters is optional (defaults to 10000)
    timeout_s: 3.0            # required
    weights:
      # heading is optional (defaults to 1.0)
      clearance: 0.75         # required
```

You can then load it in code by calling utils.config.load
```py
from rclpy.node import Node
import utils.config

class Planner(Node):
    def __init__(self):
        self.config = utils.config.load(self, PlannerConfig)
        print(self.config.max_iters)
        print(self.config.timeout_s)
        print(self.config.weights.clearance)
        print(self.config.weights.heading)
```

## utils.qos
Shared QoS profiles for use across nodes.

### `LATCHED`
`RELIABLE` + `TRANSIENT_LOCAL` + `KEEP_LAST` (depth 1). Use this for topics where late-joining subscribers must receive the last published message immediately on connect (e.g. ground truth map, robot state, GPS waypoint).

Both the publisher **and** subscriber must use the same profile — a mismatch silently drops all messages. Always use `utils.qos.LATCHED` on both sides rather than constructing the profile inline.

```py
import utils.qos

# publisher
self.create_publisher(String, "state", utils.qos.LATCHED)

# subscriber
self.create_subscription(String, "state", self.callback, utils.qos.LATCHED)
```

In RViz, set the topic's **Durability Policy** to `Transient Local` to receive latched messages.

## utils.math
Math utilities.

| Function | Description |
|---|---|
| `clamp(value, *, min, max)` | Clamp `value` to `[min, max]`. `min` and `max` are keyword-only. |

## utils.geometry
2D geometry types and helpers for ROS 2.

### `Rotation2d`
Wraps a yaw angle in radians. Angle is automatically wrapped to `[-π, π]` and `cos`/`sin` are cached on construction.

| Method | Description |
|---|---|
| `rotate_by(rotation)` on `Point2d` | Rotate a point forward by this rotation |
| `__neg__` | Negate the rotation |
| `__add__`, `__sub__` | Compose rotations by adding/subtracting angles |
| `__mul__(scalar)`, `__rmul__(scalar)`, `__truediv__(scalar)` | Scale the angle by a scalar |
| `cos`, `sin` | Cached cosine and sine of the angle |
| `to_ros()` | Convert to `geometry_msgs/Quaternion` |
| `from_ros(q)` | Construct from a `geometry_msgs/Quaternion` |
| `from_vector(vector)` | Construct from a 2D direction vector via `atan2(y, x)` |

### `Point2d`
2D point with arithmetic operators and ROS interop.

| Method | Description |
|---|---|
| `__add__`, `__sub__` | Point addition and subtraction |
| `__mul__`, `__rmul__`, `__truediv__` | Scalar multiplication and division |
| `__neg__` | Negate both components |
| `rotate_by(rotation)` | Rotate by a `Rotation2d` |
| `mag()` | Euclidean magnitude |
| `dot(other)` | Dot product |
| `cross(other)` | 2D cross product (scalar): `x·other.y − y·other.x`. Positive when `other` is to the left of `self`. |
| `lerp(other, t)` | Linearly interpolate toward `other`: `self + (other - self) * t`. `t=0` returns `self`, `t=1` returns `other`. Not clamped. |
| `distance(other)` | Distance to another `Point2d` |
| `to_ros()` | Convert to `geometry_msgs/Point` |
| `from_ros(point)` | Construct from a `geometry_msgs/Point` |

### `Pose2d`
2D pose (position + rotation) with world/local frame transforms and ROS interop.

| Method | Description |
|---|---|
| `world_to_local(world_point)` | Transform a world-frame `Point2d` into this pose's local frame |
| `local_to_world(local_point)` | Transform a local-frame `Point2d` back into world frame |
| `to_ros()` | Convert to `geometry_msgs/Pose` |
| `from_ros(pose)` | Construct from a `geometry_msgs/Pose` |

### `Path2d`
An ordered sequence of 2D waypoints. Requires at least 2 points with no consecutive duplicates (all segments must be non-zero length).

| Method | Description |
|---|---|
| `__len__()` | Number of waypoints |
| `__bool__()` | `True` if the path has any waypoints |
| `__iter__()` | Iterate over waypoints in order |
| `__getitem__(index)` | Waypoint at an integer index (supports negative indexing), or interpolated point for a float fractional index (integer part = segment, fractional part = interpolation parameter) |
| `project(point, start_index)` | Returns the fractional path index of the closest point on the path to `point`, searching forward from `start_index`. Returns `None` if `start_index` is at or past the last waypoint. On ties, prefers the later segment to commit forward at vertices. |
| `advance(start_index, distance)` | Walk forward `distance` meters along the path from a fractional index. Returns the last waypoint if the path ends before `distance` is consumed. |
| `from_ros(path)` | Construct from a `nav_msgs/Path` message, discarding z and pose orientations |

## utils.world_occupancy_grid
This class provides a world-coordinate view of a discrete, robot-centric occupancy grid.

It allows planners to operate entirely on world points - querying occupancy, expanding neighbors, and hashing
locations - without directly interacting with grid indices. Conceptually, the occupancy grid is treated as an infinite
world representation: world points are projected into grid cells on demand, and any point outside the underlying grid
bounds is treated as unknown.

All public methods accept either `geometry_msgs/Point` or `Point2d`. Methods that return points (`neighbors4`,
`neighbors8`, `neighbors_forward`, `in_bound_points`) preserve the caller's point type.

### Conventions / Transformations
The supplied occupancy grid is assumed to have the following conventions (matching ROS conventions):
- +x points forward from the robot
- +y points to the left of the robot
- the grid origin is the bottom-left corner of the grid
- data is row major

### State
State of the occupancy grid at some point can be queried using `state(point)`, which returns a `CellState`. Points
outside `[0..width) × [0..height)` return an unknown cell.

`CellState` exposes the following properties:

| Property | Description |
|---|---|
| `is_unknown` | True if the cell lies outside the grid bounds |
| `is_drivable` | True if occupancy probability is ≤ 30 |

### Full grid iteration in continuous space via in_bound_points
To iterate through all in-bound cells, `WorldOccupancyGrid` provides `in_bound_points(point_type)`. Pass `Point` or
`Point2d` to control the yielded type.

Example pattern:
```py
for candidate in grid.in_bound_points(Point2d):
    if grid.state(candidate).is_drivable:
        # found drivable cell, do something special
```

### Discrete “search” in continuous space via neighbors
Although planner code operates on continuous world points, discrete graph search can still be performed using
`neighbors4(point)`, `neighbors8(point)`, or `neighbors_forward(point)`.

Each neighbor expansion:
1. Projects the input world point into a grid cell
2. Expands neighboring cells in grid index space
3. Converts those neighboring cells back into world points by returning the center of each cell

The returned points are the same type as the input (`Point` or `Point2d`), so no conversion is needed at the call site.

Example pattern:
```py
for candidate in grid.neighbors8(current):  # candidate matches type of current
    if not grid.state(candidate).is_drivable:
        continue
```
### Hashing
To support discrete search bookkeeping (e.g. visited sets), `WorldOccupancyGrid` provides `hash_key(point)`, which 
returns a stable integer identifier corresponding to the grid cell that the point belongs to.

The hash is derived from the projected grid indices, ensuring that all world points falling within the same grid cell 
map to the same key. This allows planners to treat grid cells as discrete states without storing raw grid indices or 
floating-point coordinates.
