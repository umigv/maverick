import math

import numpy as np
from pointcloud_simulator.raycast import (
    ObstacleMap,
    build_ray_directions,
    cast_rays,
    quaternion_from_rpy,
    rotation_matrix,
)

RESOLUTION_M = 0.1
OBSTACLE_HEIGHT_M = 0.6


def single_cell_map(cell_x: int, cell_y: int) -> ObstacleMap:
    return ObstacleMap.from_cells([(cell_x, cell_y)], RESOLUTION_M, OBSTACLE_HEIGHT_M)


def cast_one(
    origin: tuple[float, float, float], direction: tuple[float, float, float], obstacle_map: ObstacleMap
) -> float:
    unit_direction = np.array(direction) / np.linalg.norm(direction)
    return float(cast_rays(np.array(origin), unit_direction[np.newaxis, :], obstacle_map, max_range_m=10.0)[0])


# --- Ground ---


def test_downward_ray_hits_ground_at_origin_height() -> None:
    empty_map = ObstacleMap.from_cells([], RESOLUTION_M, OBSTACLE_HEIGHT_M)
    assert math.isclose(cast_one((0.0, 0.0, 0.5), (0.0, 0.0, -1.0), empty_map), 0.5)


def test_angled_ray_hits_ground_along_hypotenuse() -> None:
    empty_map = ObstacleMap.from_cells([], RESOLUTION_M, OBSTACLE_HEIGHT_M)
    assert math.isclose(cast_one((0.0, 0.0, 1.0), (1.0, 0.0, -1.0), empty_map), math.sqrt(2))


def test_horizontal_ray_over_empty_map_hits_nothing() -> None:
    empty_map = ObstacleMap.from_cells([], RESOLUTION_M, OBSTACLE_HEIGHT_M)
    assert math.isinf(cast_one((0.0, 0.0, 0.3), (1.0, 0.0, 0.0), empty_map))


# --- Obstacles ---


def test_horizontal_ray_hits_near_face_of_obstacle() -> None:
    # Cell 10 spans x in [1.0, 1.1); the ray starts at x = 0.05 so it hits the near face 0.95 m away.
    distance = cast_one((0.05, 0.05, 0.3), (1.0, 0.0, 0.0), single_cell_map(10, 0))
    assert math.isclose(distance, 0.95)


def test_negative_direction_ray_hits_obstacle() -> None:
    # Cell -10 spans x in [-1.0, -0.9); the ray hits its far edge at x = -0.9.
    distance = cast_one((0.05, 0.05, 0.3), (-1.0, 0.0, 0.0), single_cell_map(-10, 0))
    assert math.isclose(distance, 0.95)


def test_diagonal_ray_hits_obstacle() -> None:
    distance = cast_one((0.05, 0.05, 0.3), (1.0, 1.0, 0.0), single_cell_map(5, 5))
    assert math.isclose(distance, 0.45 * math.sqrt(2))


def test_ray_above_obstacle_height_misses() -> None:
    assert math.isinf(cast_one((0.05, 0.05, 1.0), (1.0, 0.0, 0.0), single_cell_map(10, 0)))


def test_ray_descending_onto_obstacle_hits_its_top() -> None:
    # The ray starts at z = 1.0 above the cell spanning x in [1.0, 1.1) and drops to the top face (z = 0.6).
    distance = cast_one((1.05, 0.05, 1.0), (0.0, 0.0, -1.0), single_cell_map(10, 0))
    assert math.isclose(distance, 0.4)


def test_obstacle_occludes_ground_behind_it() -> None:
    # Without the obstacle this ray would hit the ground at x = 3.05.
    distance = cast_one((0.05, 0.05, 0.3), (1.0, 0.0, -0.1), single_cell_map(10, 0))
    assert math.isclose(distance, 0.95 * math.sqrt(1 + 0.1**2))


def test_obstacle_beyond_max_range_is_ignored() -> None:
    obstacle_map = single_cell_map(200, 0)
    distances = cast_rays(np.array([0.05, 0.05, 0.3]), np.array([[1.0, 0.0, 0.0]]), obstacle_map, max_range_m=5.0)
    assert math.isinf(distances[0])


# --- Ray directions ---


def test_full_circle_has_no_duplicate_azimuth() -> None:
    directions = build_ray_directions(2 * math.pi, math.radians(10), math.radians(1), math.radians(10))
    azimuths = np.round(np.arctan2(directions[:, 1], directions[:, 0]), 9)
    elevations = np.round(np.arcsin(directions[:, 2]), 9)
    assert len(directions) == 360 * 2
    assert len(set(zip(azimuths, elevations, strict=True))) == len(directions)


def test_partial_fov_spans_both_edges() -> None:
    directions = build_ray_directions(math.radians(90), math.radians(60), math.radians(1), math.radians(1))
    azimuths = np.arctan2(directions[:, 1], directions[:, 0])
    elevations = np.arcsin(directions[:, 2])
    assert math.isclose(azimuths.min(), -math.pi / 4) and math.isclose(azimuths.max(), math.pi / 4)
    assert math.isclose(elevations.min(), -math.pi / 6) and math.isclose(elevations.max(), math.pi / 6)
    assert np.allclose(np.linalg.norm(directions, axis=1), 1.0)


# --- Rotations ---


def test_positive_pitch_points_down_and_positive_yaw_points_left() -> None:
    assert np.allclose(rotation_matrix(0.0, math.pi / 2, 0.0) @ [1.0, 0.0, 0.0], [0.0, 0.0, -1.0])
    assert np.allclose(rotation_matrix(0.0, 0.0, math.pi / 2) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])


def test_quaternion_matches_rotation_matrix() -> None:
    roll, pitch, yaw = 0.3, -0.2, 1.1
    x, y, z, w = quaternion_from_rpy(roll, pitch, yaw)
    from_quaternion = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    assert np.allclose(from_quaternion, rotation_matrix(roll, pitch, yaw))
