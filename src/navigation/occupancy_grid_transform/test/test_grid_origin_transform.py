import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from occupancy_grid_transform.occupancy_grid_transform import transform_grid_origin
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException

# These tests feed transform_grid_origin a fake TF history (no ROS node needed) and check where the grid ends up in
# odom. It returns (origin, placed_at_stamp): placed_at_stamp is False when it had to fall back to the latest transform.


def odom_to_base_link(x: float, sec: int) -> TransformStamped:
    """Return the robot at (x, 0) facing +x, at time sec."""
    transform = TransformStamped(header=Header(frame_id="odom", stamp=Time(sec=sec)), child_frame_id="base_link")
    transform.transform.translation.x = x
    transform.transform.rotation.w = 1.0
    return transform


def moving_robot_buffer() -> Buffer:
    """Return a buffer in which the robot drives 1 m forward between t = 10 s and t = 11 s."""
    buffer = Buffer()
    buffer.set_transform(odom_to_base_link(0.0, 10), "test")  # "test" is just the authority name tf wants
    buffer.set_transform(odom_to_base_link(1.0, 11), "test")
    return buffer


def grid_stamped(sec: int, nanosec: int = 0) -> OccupancyGrid:
    """Return a robot-centric grid (in base_link) captured at the given time. Only the origin matters for these tests."""
    origin = Pose()
    origin.position.y = -2.5  # grid corner 2.5 m to the robot's right, like a 5 m wide grid centered on the robot
    origin.orientation.w = 1.0
    return OccupancyGrid(
        header=Header(frame_id="base_link", stamp=Time(sec=sec, nanosec=nanosec)), info=MapMetaData(origin=origin)
    )


def test_grid_is_placed_with_transform_at_its_stamp() -> None:
    # Captured at t = 10.5 s, halfway through the drive, but transformed after the robot has reached x = 1
    origin, placed_at_stamp = transform_grid_origin(moving_robot_buffer(), grid_stamped(10, 500_000_000), "odom")

    assert placed_at_stamp
    assert origin.position.x == pytest.approx(0.5)  # where the robot was at capture, not the latest 1.0
    assert origin.position.y == pytest.approx(-2.5)


def test_grid_stamped_just_ahead_of_odometry_uses_latest_transform_without_fallback() -> None:
    # Stamped 30 ms after the newest odometry: normal timing jitter, within FUTURE_STAMP_TOLERANCE_S, so no warning
    origin, placed_at_stamp = transform_grid_origin(moving_robot_buffer(), grid_stamped(11, 30_000_000), "odom")

    assert placed_at_stamp
    assert origin.position.x == pytest.approx(1.0)


@pytest.mark.parametrize("stamp_sec", [12, 5])  # 1 s ahead of the latest transform, older than the oldest one
def test_grid_stamped_far_from_odometry_falls_back_to_latest_transform(stamp_sec: int) -> None:
    # No transform exists near the stamp, so it uses the latest one and reports the fallback (the node logs a warning)
    origin, placed_at_stamp = transform_grid_origin(moving_robot_buffer(), grid_stamped(stamp_sec), "odom")

    assert not placed_at_stamp
    assert origin.position.x == pytest.approx(1.0)


def test_missing_frame_raises() -> None:
    # There's no "map" frame in the buffer at all, so falling back to the latest transform can't help either
    with pytest.raises(TransformException):
        transform_grid_origin(moving_robot_buffer(), grid_stamped(10), "map")
