import heapq
import math
from dataclasses import dataclass, field

from utils.geometry import Point2d, Pose2d
from utils.world_occupancy_grid import WorldOccupancyGrid

from .path_planning_config import PathPlanningConfig


@dataclass(order=True)
class KeyAndCost:
    """Store the coordinates to a node and the cost of that node."""

    cost: float
    key: int = field(compare=False)


def generate_path(
    grid: WorldOccupancyGrid,
    robot_pose: Pose2d,
    goal: Point2d,
    config: PathPlanningConfig,
) -> list[Point2d] | None:
    """Generate a good path for the robot to follow towards the goal using the A* search algorithm.
    https://en.wikipedia.org/wiki/A*_search_algorithm.

    Unknown cells within the asymmetric forward/sideways bounds around the start are treated as traversable, allowing A*
    to plan directly from the robot's position even when the cells immediately under it are unknown in the grid. If the
    goal is unreachable (e.g. blocked by obstacles), the path leads to the closest reachable cell instead.

    Args:
        grid: World-coordinate occupancy grid.
        robot_pose: Robot pose (position + heading) in world coordinates.
        goal: Goal point in world coordinates.
        config: Path planning configuration.
    Returns:
        List of world-coordinate points from start to goal (or closest
        reachable point), or None if no drivable cells are reachable.
    """
    start = robot_pose.point
    start_key = grid.hash_key(start)
    goal_key = grid.hash_key(goal)

    if start_key == goal_key:
        return [start]

    point_of: dict[int, Point2d] = {start_key: start}
    came_from: dict[int, int | None] = {start_key: None}
    cost_so_far: dict[int, float] = {start_key: 0.0}
    priority_queue: list[KeyAndCost] = [KeyAndCost(start.distance(goal), start_key)]

    best_goal_key = start_key
    best_goal_distance = start.distance(goal)

    while len(priority_queue) > 0:
        current_key = heapq.heappop(priority_queue).key
        current_point = point_of[current_key]

        if current_point.distance(goal) < best_goal_distance:
            best_goal_key = current_key
            best_goal_distance = current_point.distance(goal)

        if current_key == goal_key:
            best_goal_key = current_key
            break

        for neighbor in grid.neighbors8(current_point):
            state = grid.state(neighbor)
            if state.is_unknown:
                local = robot_pose.world_to_local(neighbor)
                if not (0 <= local.x <= config.max_unknown_forward_m) or abs(local.y) > config.max_unknown_sideways_m:
                    continue
            elif not state.is_drivable:
                continue

            neighbor_key = grid.hash_key(neighbor)
            neighbor_cost = cost_so_far[current_key] + current_point.distance(neighbor)

            if neighbor_key not in cost_so_far or neighbor_cost < cost_so_far[neighbor_key]:
                cost_so_far[neighbor_key] = neighbor_cost
                came_from[neighbor_key] = current_key
                point_of[neighbor_key] = neighbor
                priority = neighbor_cost + neighbor.distance(goal)
                heapq.heappush(priority_queue, KeyAndCost(priority, neighbor_key))

    if best_goal_key == start_key:
        return None

    backtrace: list[Point2d] = []
    backtrace_key: int | None = best_goal_key
    while backtrace_key is not None:
        backtrace.append(point_of[backtrace_key])
        backtrace_key = came_from[backtrace_key]

    return list(reversed(backtrace))


def pull_string(
    grid: WorldOccupancyGrid,
    path: list[Point2d],
    robot_pose: Pose2d,
    config: PathPlanningConfig,
) -> list[Point2d]:
    """Remove unnecessary waypoints from a path using the string-pulling algorithm.
    https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html

    From each waypoint, finds the furthest subsequent waypoint reachable in a straight
    line and skips everything in between. This eliminates the staircase artifacts
    produced by grid-based A* and reduces sharp heading changes at the start of the path.

    Args:
        grid: World-coordinate occupancy grid.
        path: Input path from A*.
        robot_pose: Robot pose used to evaluate unknown-cell traversal bounds.
        config: Path planning configuration.
    Returns:
        Pruned path containing only waypoints where direction changes are necessary.
    """

    def line_of_sight(a: Point2d, b: Point2d) -> bool:
        distance = a.distance(b)
        if distance < 1e-9:
            return True

        steps = max(2, math.ceil(distance / config.line_of_sight_step_m))
        for i in range(1, steps):
            p = a + (b - a) * (i / steps)
            state = grid.state(p)
            if state.is_unknown:
                local = robot_pose.world_to_local(p)
                if not (0 <= local.x <= config.max_unknown_forward_m) or abs(local.y) > config.max_unknown_sideways_m:
                    return False
            elif not state.is_drivable:
                return False

        return True

    if len(path) <= 2:
        return path

    result = [path[0]]
    anchor = 0

    while anchor < len(path) - 1:
        furthest = anchor + 1
        for i in range(anchor + 2, len(path)):
            if line_of_sight(path[anchor], path[i]):
                furthest = i
        result.append(path[furthest])
        anchor = furthest

    return result
