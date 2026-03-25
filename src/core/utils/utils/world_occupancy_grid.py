from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass
from typing import ClassVar, TypeVar

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from utils.geometry import Point2d, Pose2d

PointT = TypeVar("PointT", Point, Point2d)


@dataclass(frozen=True)
class CellState:
    """
    Discrete occupancy state of a grid cell.
    """

    value: int  # -1 (unknown) or 0-100 (probability occupied)

    UNKNOWN_VALUE: ClassVar[int] = -1
    DRIVABLE_THRESHOLD: ClassVar[int] = 30

    def __post_init__(self) -> None:
        if not (0 <= self.value <= 100 or self.value == CellState.UNKNOWN_VALUE):
            raise ValueError("CellState value must be within [0, 100] or equal to CellState.UNKNOWN_VALUE")

    @classmethod
    def unknown_cell(cls) -> CellState:
        return cls(cls.UNKNOWN_VALUE)

    @property
    def is_drivable(self) -> bool:
        """
        True if the cell can be traversed by the robot
        """
        return 0 <= self.value <= CellState.DRIVABLE_THRESHOLD

    @property
    def is_unknown(self) -> bool:
        """
        True if the cell value is unknown
        """
        return self.value == CellState.UNKNOWN_VALUE


class WorldOccupancyGrid:
    """
    Continuous world-facing view of an occupancy grid.

    This class wraps a discrete `nav_msgs/msg/OccupancyGrid` and exposes it as a continuous world-coordinate
    representation. This abstraction allows discrete grid-based search (e.g. A*, BFS) to be expressed entirely in world
    coordinates, while preserving correct grid semantics.

    Conceptually, the occupancy grid is treated as an infinite world:
    - World points are projected into grid cells on demand.
    - Points outside the underlying grid bounds are treated as UNKNOWN.

    Attributes:
        _occupancy_grid: The occupancy grid.
        _origin: Pose2d representing the origin of the occupancy grid in world coordinates.
    """

    _occupancy_grid: OccupancyGrid
    _origin: Pose2d

    def __init__(self, grid: OccupancyGrid) -> None:
        """
        Construct a world-facing view of a discrete occupancy grid.

        The occupancy grid is assumed to be in ROS2 conventions, where
        - +x is forward
        - +y is left
        - data is stored in row major order

        Args:
            grid: Occupancy grid message.
        """
        self._occupancy_grid = grid
        self._origin = Pose2d.from_ros(self._occupancy_grid.info.origin)

    def state(self, point: Point | Point2d) -> CellState:
        """
        Query the occupancy state at a world-coordinate point.

        The point is projected into the underlying occupancy grid. If the projected grid index lies outside the grid
        bounds, the state is treated as UNKNOWN.

        Args:
            point: World-coordinate point to query.

        Returns:
            CellState corresponding to the occupancy at the given point.
        """
        grid_x, grid_y = self._world_to_grid_index(point)

        if not (0 <= grid_x < self._occupancy_grid.info.width and 0 <= grid_y < self._occupancy_grid.info.height):
            return CellState.unknown_cell()

        return CellState(self._occupancy_grid.data[grid_y * self._occupancy_grid.info.width + grid_x])

    def in_bound_points(self, point_type: type[PointT]) -> Iterator[PointT]:
        """
        Generate a point in all in bound grids

        Args:
            point_type: The point type to yield - either Point or Point2d.

        Yields:
            World-coordinate points at the centers of all in-bound grid cells.
        """
        for x in range(self._occupancy_grid.info.width):
            for y in range(self._occupancy_grid.info.height):
                yield self._grid_index_center_to_world(x, y, point_type)

    def neighbors4(self, point: PointT) -> Iterator[PointT]:
        """
        Generate 4-connected neighboring world points for discrete grid search.

        Neighbors correspond to the centers of the grid cells adjacent to the cell containing `point` in the cardinal
        directions (±X, ±Y).

        Args:
            point: World-coordinate point whose grid cell is expanded.

        Yields:
            World-coordinate points at the centers of neighboring grid cells, in the same type as `point`.
        """
        x, y = self._world_to_grid_index(point)

        for dx, dy in ((1, 0), (0, 1), (0, -1), (-1, 0)):
            yield self._grid_index_center_to_world(x + dx, y + dy, type(point))

    def neighbors8(self, point: PointT) -> Iterator[PointT]:
        """
        Generate 8-connected neighboring world points for discrete grid search.

        Neighbors correspond to the centers of all adjacent grid cells, including diagonals, around the cell containing
        `point`.

        Args:
            point: World-coordinate point whose grid cell is expanded.

        Yields:
            World-coordinate points at the centers of neighboring grid cells, in the same type as `point`.
        """
        x, y = self._world_to_grid_index(point)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield self._grid_index_center_to_world(x + dx, y + dy, type(point))

    def neighbors_forward(self, point: PointT) -> Iterator[PointT]:
        """
        Generate a forward-biased set of neighboring world points.

        This expansion favors motion in front of the robot and is useful for planners that prefer forward progress.
        The expansion includes: forward, forward-left, forward-right, left, and right neighbors.

        Args:
            point: World-coordinate point whose grid cell is expanded.

        Yields:
            World-coordinate points at the centers of selected neighboring cells, in the same type as `point`.
        """
        x, y = self._world_to_grid_index(point)

        candidates = [
            (1, 0),  # forward
            (1, 1),  # forward-left
            (1, -1),  # forward-right
            (0, 1),  # left
            (0, -1),  # right
        ]

        for dx, dy in candidates:
            yield self._grid_index_center_to_world(x + dx, y + dy, type(point))

    def hash_key(self, point: Point | Point2d) -> int:
        """
        Compute a stable hash key for the grid cell containing a world point.

        All world points that project into the same grid cell will produce the same hash key. This enables discrete
        search bookkeeping (e.g. visited sets) without storing raw grid indices or floating-point coordinates.

        Args:
            point: World-coordinate point to hash.

        Returns:
            Integer hash uniquely identifying the corresponding grid cell.
        """
        x, y = self._world_to_grid_index(point)

        # map signed to unsigned by mapping positive to even and negative to odd
        zx = 2 * x if x >= 0 else -2 * x - 1
        zy = 2 * y if y >= 0 else -2 * y - 1

        # cantor pairing (https://en.wikipedia.org/wiki/Pairing_function)
        return (zx + zy) * (zx + zy + 1) // 2 + zy

    def _world_to_grid_index(self, world: Point | Point2d) -> tuple[int, int]:
        """
        Project a world-coordinate point into discrete grid indices.

        Args:
            world: World-coordinate point.

        Returns:
            (grid_x, grid_y) integer indices corresponding to the grid cell containing the point.
        """
        p2d = Point2d.from_ros(world) if isinstance(world, Point) else world
        grid = self._origin.world_to_local(p2d) / self._occupancy_grid.info.resolution
        return math.floor(grid.x), math.floor(grid.y)

    def _grid_index_center_to_world(self, grid_x: int, grid_y: int, point_type: type[PointT]) -> PointT:
        """
        Convert a grid cell index to the world-coordinate position of its center.

        This is the inverse of `_world_to_grid_index` (up to quantization), mapping discrete grid indices back into
        continuous world space.

        Args:
            grid_x: Grid index in the +X (forward) direction.
            grid_y: Grid index in the +Y (left) direction.
            point_type: The point type to return - either Point or Point2d.

        Returns:
            World-coordinate point at the center of the specified grid cell.
        """
        grid = Point2d(x=grid_x + 0.5, y=grid_y + 0.5) * self._occupancy_grid.info.resolution
        p2d = self._origin.local_to_world(grid)
        return p2d.to_ros() if point_type is Point else p2d
