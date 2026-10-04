"""Pure numpy raycasting against a ground plane and a 2D obstacle map extruded to a fixed height."""

import math
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

EPSILON = 1e-12


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for intrinsic ZYX (yaw, then pitch, then roll) Euler angles."""
    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    roll_matrix = np.array([[1.0, 0.0, 0.0], [0.0, cos_roll, -sin_roll], [0.0, sin_roll, cos_roll]])
    pitch_matrix = np.array([[cos_pitch, 0.0, sin_pitch], [0.0, 1.0, 0.0], [-sin_pitch, 0.0, cos_pitch]])
    yaw_matrix = np.array([[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]])
    rotation: np.ndarray = yaw_matrix @ pitch_matrix @ roll_matrix
    return rotation


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return the (x, y, z, w) quaternion matching `rotation_matrix(roll, pitch, yaw)`."""
    cos_half_roll, sin_half_roll = math.cos(roll / 2), math.sin(roll / 2)
    cos_half_pitch, sin_half_pitch = math.cos(pitch / 2), math.sin(pitch / 2)
    cos_half_yaw, sin_half_yaw = math.cos(yaw / 2), math.sin(yaw / 2)

    return (
        sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw,
        cos_half_roll * sin_half_pitch * cos_half_yaw + sin_half_roll * cos_half_pitch * sin_half_yaw,
        cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw,
        cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw,
    )


def build_ray_directions(
    horizontal_fov_rad: float,
    vertical_fov_rad: float,
    horizontal_resolution_rad: float,
    vertical_resolution_rad: float,
) -> np.ndarray:
    """
    Return an (N, 3) array of unit ray directions in the sensor frame, centered on the sensor's +x axis.

    A full-circle horizontal FOV skips the duplicate ray at +pi so azimuths are evenly spaced around the circle.
    """
    if math.isclose(horizontal_fov_rad, 2 * math.pi):
        horizontal_count = max(1, round(horizontal_fov_rad / horizontal_resolution_rad))
        azimuths = -math.pi + np.arange(horizontal_count) * (2 * math.pi / horizontal_count)
    else:
        horizontal_count = round(horizontal_fov_rad / horizontal_resolution_rad) + 1
        azimuths = np.linspace(-horizontal_fov_rad / 2, horizontal_fov_rad / 2, horizontal_count)

    vertical_count = round(vertical_fov_rad / vertical_resolution_rad) + 1
    elevations = np.linspace(-vertical_fov_rad / 2, vertical_fov_rad / 2, vertical_count)

    azimuth_grid, elevation_grid = np.meshgrid(azimuths, elevations)
    azimuth_grid = azimuth_grid.ravel()
    elevation_grid = elevation_grid.ravel()
    return np.column_stack(
        (
            np.cos(elevation_grid) * np.cos(azimuth_grid),
            np.cos(elevation_grid) * np.sin(azimuth_grid),
            np.sin(elevation_grid),
        )
    )


@dataclass(frozen=True)
class ObstacleMap:
    """
    Dense boolean obstacle grid. Each occupied cell is a box spanning `[0, obstacle_height_m]` in z.

    `occupied[row, column]` corresponds to world cell `(min_cell_x + column, min_cell_y + row)`. Cells outside the
    array are free.
    """

    occupied: np.ndarray
    min_cell_x: int
    min_cell_y: int
    resolution_m: float
    obstacle_height_m: float

    @classmethod
    def from_cells(cls, cells: Iterable[tuple[int, int]], resolution_m: float, obstacle_height_m: float) -> ObstacleMap:
        cell_array = np.array(list(cells), dtype=np.int64).reshape(-1, 2)
        if len(cell_array) == 0:
            return cls(np.zeros((0, 0), dtype=bool), 0, 0, resolution_m, obstacle_height_m)

        min_cell_x, min_cell_y = cell_array.min(axis=0)
        max_cell_x, max_cell_y = cell_array.max(axis=0)
        occupied = np.zeros((max_cell_y - min_cell_y + 1, max_cell_x - min_cell_x + 1), dtype=bool)
        occupied[cell_array[:, 1] - min_cell_y, cell_array[:, 0] - min_cell_x] = True
        return cls(occupied, int(min_cell_x), int(min_cell_y), resolution_m, obstacle_height_m)

    def is_occupied(self, cell_x: np.ndarray, cell_y: np.ndarray) -> np.ndarray:
        rows = cell_y - self.min_cell_y
        columns = cell_x - self.min_cell_x
        height, width = self.occupied.shape
        in_bounds = (rows >= 0) & (rows < height) & (columns >= 0) & (columns < width)

        result = np.zeros(cell_x.shape, dtype=bool)
        result[in_bounds] = self.occupied[rows[in_bounds], columns[in_bounds]]
        return result


def ground_distances(origin: np.ndarray, directions: np.ndarray) -> np.ndarray:
    """Return the distance along each ray to the z = 0 plane, or inf for rays that never reach it."""
    direction_z = directions[:, 2]
    downward = direction_z < -EPSILON
    distances = np.full(len(directions), np.inf)
    distances[downward] = -origin[2] / direction_z[downward]
    return distances


def obstacle_height_interval(
    origin_z: float, direction_z: np.ndarray, obstacle_height_m: float
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return the distance interval `(lower_distance, upper_distance)` over which each ray has `0 <= z <= height`.

    Rays parallel to the ground get an unbounded interval if they start inside the height band, or an empty interval
    otherwise.
    """
    parallel = np.abs(direction_z) <= EPSILON
    inside = 0.0 <= origin_z <= obstacle_height_m

    safe_direction_z = np.where(parallel, 1.0, direction_z)
    ground_distance = -origin_z / safe_direction_z
    top_distance = (obstacle_height_m - origin_z) / safe_direction_z

    lower_distance = np.where(parallel, -np.inf if inside else np.inf, np.minimum(ground_distance, top_distance))
    upper_distance = np.where(parallel, np.inf if inside else -np.inf, np.maximum(ground_distance, top_distance))
    return lower_distance, upper_distance


def obstacle_distances(
    origin: np.ndarray,
    directions: np.ndarray,
    obstacle_map: ObstacleMap,
    max_range_m: float,
    stop_distances: np.ndarray,
) -> np.ndarray:
    """
    Return the distance along each ray to the first obstacle surface, or inf if none is hit.

    Walks every ray through the 2D grid at once with a vectorized Amanatides-Woo DDA. In each cell the ray visits, it
    hits if the cell is occupied and the ray's z is within the obstacle height somewhere inside that cell. A ray stops
    walking once it passes `max_range_m`, its `stop_distances` entry (e.g. where it hits the ground), or the top of the
    obstacle band.
    """
    ray_count = len(directions)
    resolution = obstacle_map.resolution_m
    origin_x, origin_y, origin_z = origin
    direction_x = directions[:, 0]
    direction_y = directions[:, 1]

    lower_height_distance, upper_height_distance = obstacle_height_interval(
        origin_z, directions[:, 2], obstacle_map.obstacle_height_m
    )

    def axis_setup(
        origin_coordinate: float, direction: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return (cell, step, distance to the first cell boundary, distance between boundaries) along one axis."""
        start_cell = math.floor(origin_coordinate / resolution)
        cell = np.full(ray_count, start_cell, dtype=np.int64)
        step = np.sign(direction).astype(np.int64)
        moving = np.abs(direction) > EPSILON
        safe_direction = np.where(moving, direction, 1.0)

        next_boundary = np.where(direction > 0, (start_cell + 1) * resolution, start_cell * resolution)
        next_boundary_distance = np.where(moving, (next_boundary - origin_coordinate) / safe_direction, np.inf)
        boundary_spacing = np.where(moving, resolution / np.abs(safe_direction), np.inf)
        return cell, step, next_boundary_distance, boundary_spacing

    cell_x, step_x, next_boundary_distance_x, boundary_spacing_x = axis_setup(origin_x, direction_x)
    cell_y, step_y, next_boundary_distance_y, boundary_spacing_y = axis_setup(origin_y, direction_y)

    hit_distances = np.full(ray_count, np.inf)
    enter_distance = np.zeros(ray_count)
    max_walk_distance = np.minimum(np.minimum(stop_distances, upper_height_distance), max_range_m)
    active = lower_height_distance <= max_walk_distance

    # A ray crosses at most one x boundary and one y boundary per cell of travel along each axis.
    max_steps = math.ceil(2 * max_range_m / resolution) + 2
    for _ in range(max_steps):
        if not active.any():
            break

        exit_distance = np.minimum(next_boundary_distance_x, next_boundary_distance_y)

        candidates = active & obstacle_map.is_occupied(cell_x, cell_y)
        if candidates.any():
            hit_start = np.maximum(enter_distance, lower_height_distance)
            hit_end = np.minimum(exit_distance, upper_height_distance)
            hits = candidates & (hit_start <= hit_end)
            hit_distances[hits] = hit_start[hits]
            active &= ~hits

        step_along_x = next_boundary_distance_x < next_boundary_distance_y
        step_along_y = ~step_along_x
        cell_x = np.where(step_along_x, cell_x + step_x, cell_x)
        cell_y = np.where(step_along_y, cell_y + step_y, cell_y)
        next_boundary_distance_x = np.where(
            step_along_x, next_boundary_distance_x + boundary_spacing_x, next_boundary_distance_x
        )
        next_boundary_distance_y = np.where(
            step_along_y, next_boundary_distance_y + boundary_spacing_y, next_boundary_distance_y
        )
        enter_distance = exit_distance
        active &= enter_distance <= max_walk_distance

    return hit_distances


def cast_rays(origin: np.ndarray, directions: np.ndarray, obstacle_map: ObstacleMap, max_range_m: float) -> np.ndarray:
    """
    Return the distance along each unit ray from `origin` to the nearest surface (ground or obstacle).

    Rays that hit nothing get inf. Distances beyond `max_range_m` may still be returned for ground hits; callers should
    filter by range.
    """
    ground = ground_distances(origin, directions)
    obstacles = obstacle_distances(origin, directions, obstacle_map, max_range_m, ground)
    distances: np.ndarray = np.minimum(ground, obstacles)
    return distances
