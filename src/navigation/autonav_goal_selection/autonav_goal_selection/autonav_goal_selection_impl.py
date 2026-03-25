import math

from utils.geometry import Point2d, Pose2d
from utils.world_occupancy_grid import WorldOccupancyGrid

from .autonav_goal_selection_config import GoalSelectionParams


def select_goal(
    grid: WorldOccupancyGrid, robot_pose: Pose2d, waypoint: Point2d, params: GoalSelectionParams
) -> Point2d | None:
    """Select the best drivable goal in the occupancy grid.

    Scores every in-bounds drivable cell using a heuristic and returns
    the highest-scoring point. Non-drivable cells are excluded.

    Args:
        grid: World-coordinate occupancy grid.
        robot_pose: Robot pose in world coordinates.
        waypoint: Target waypoint in world coordinates.
        params: Goal selection algorithm parameters.
    Returns:
        The best drivable goal point, or None if no drivable cells exist.
    """

    def heuristic(point: Point2d) -> float:
        if not grid.state(point).is_drivable:
            return math.inf

        # This is a bad heuristic but it works fine for open fields
        return point.distance(waypoint)

    best_point = min(grid.in_bound_points(Point2d), key=heuristic)
    return best_point if heuristic(best_point) < math.inf else None
