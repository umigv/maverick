import math

from utils.geometry import Point2d, Pose2d, Rotation2d

# --- Rotation2d ---


def test_rotation2d_wraps_above_pi():
    r = Rotation2d(math.pi + 0.1)
    assert abs(r.angle - (-math.pi + 0.1)) < 1e-9


def test_rotation2d_wraps_below_neg_pi():
    r = Rotation2d(-math.pi - 0.1)
    assert abs(r.angle - (math.pi - 0.1)) < 1e-9


def test_rotation2d_caches_cos_sin():
    r = Rotation2d(math.pi / 3)
    assert abs(r._cos - math.cos(math.pi / 3)) < 1e-12
    assert abs(r._sin - math.sin(math.pi / 3)) < 1e-12


def test_rotation2d_neg():
    r = Rotation2d(math.pi / 4)
    assert abs((-r).angle + math.pi / 4) < 1e-9


def test_rotation2d_add():
    a = Rotation2d(math.pi / 4)
    b = Rotation2d(math.pi / 4)
    assert abs((a + b).angle - math.pi / 2) < 1e-9


def test_rotation2d_sub():
    a = Rotation2d(math.pi / 2)
    b = Rotation2d(math.pi / 4)
    assert abs((a - b).angle - math.pi / 4) < 1e-9


def test_rotation2d_mul_scalar():
    r = Rotation2d(math.pi / 4)
    assert abs((r * 2.0).angle - math.pi / 2) < 1e-9


def test_rotation2d_rmul_scalar():
    r = Rotation2d(math.pi / 4)
    assert abs((2.0 * r).angle - math.pi / 2) < 1e-9


def test_rotation2d_div_scalar():
    r = Rotation2d(math.pi / 2)
    assert abs((r / 2.0).angle - math.pi / 4) < 1e-9


def test_rotation2d_to_ros_zero():
    q = Rotation2d(0.0).to_ros()
    assert abs(q.z - 0.0) < 1e-12
    assert abs(q.w - 1.0) < 1e-12


def test_rotation2d_to_ros_unit_quaternion():
    q = Rotation2d(1.234).to_ros()
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    assert abs(norm - 1.0) < 1e-12


def test_rotation2d_from_ros_round_trip():
    r = Rotation2d(math.pi / 3)
    assert abs(Rotation2d.from_ros(r.to_ros()).angle - r.angle) < 1e-6


def test_rotation2d_from_ros_round_trip_negative():
    r = Rotation2d(-2.1)
    assert abs(Rotation2d.from_ros(r.to_ros()).angle - r.angle) < 1e-6


# --- Point2d ---


def test_point2d_neg():
    p = Point2d(x=1.0, y=-2.0)
    assert -p == Point2d(x=-1.0, y=2.0)


def test_point2d_add():
    assert Point2d(1.0, 2.0) + Point2d(3.0, 4.0) == Point2d(4.0, 6.0)


def test_point2d_sub():
    assert Point2d(3.0, 4.0) - Point2d(1.0, 1.0) == Point2d(2.0, 3.0)


def test_point2d_mul():
    assert Point2d(2.0, 3.0) * 2.0 == Point2d(4.0, 6.0)


def test_point2d_rmul():
    assert 2.0 * Point2d(2.0, 3.0) == Point2d(4.0, 6.0)


def test_point2d_div():
    assert Point2d(4.0, 6.0) / 2.0 == Point2d(2.0, 3.0)


def test_point2d_rotate_by_zero():
    p = Point2d(x=1.0, y=2.0)
    r = p.rotate_by(Rotation2d(0.0))
    assert abs(r.x - 1.0) < 1e-9
    assert abs(r.y - 2.0) < 1e-9


def test_point2d_rotate_by_90():
    p = Point2d(x=1.0, y=0.0)
    r = p.rotate_by(Rotation2d(math.pi / 2))
    assert abs(r.x - 0.0) < 1e-9
    assert abs(r.y - 1.0) < 1e-9


def test_point2d_rotate_preserves_magnitude():
    p = Point2d(x=3.0, y=4.0)
    r = p.rotate_by(Rotation2d(1.2345))
    assert abs(p.mag() - r.mag()) < 1e-9


def test_point2d_mag():
    assert abs(Point2d(x=3.0, y=4.0).mag() - 5.0) < 1e-12


def test_point2d_distance():
    assert abs(Point2d(0.0, 0.0).distance(Point2d(3.0, 4.0)) - 5.0) < 1e-12


def test_point2d_distance_symmetric():
    a = Point2d(1.0, 2.0)
    b = Point2d(-3.0, 4.0)
    assert abs(a.distance(b) - b.distance(a)) < 1e-12


def test_point2d_ros_round_trip():
    p = Point2d(x=1.5, y=-2.5)
    assert Point2d.from_ros(p.to_ros()) == p


# --- Pose2d ---


def test_pose2d_world_to_local_at_origin():
    pose = Pose2d(Point2d(0.0, 0.0), Rotation2d(0.0))
    local = pose.world_to_local(Point2d(3.0, 4.0))
    assert abs(local.x - 3.0) < 1e-9
    assert abs(local.y - 4.0) < 1e-9


def test_pose2d_world_to_local_translation_only():
    pose = Pose2d(Point2d(1.0, 2.0), Rotation2d(0.0))
    local = pose.world_to_local(Point2d(4.0, 6.0))
    assert abs(local.x - 3.0) < 1e-9
    assert abs(local.y - 4.0) < 1e-9


def test_pose2d_world_to_local_rotation_only():
    pose = Pose2d(Point2d(0.0, 0.0), Rotation2d(math.pi / 2))
    local = pose.world_to_local(Point2d(1.0, 0.0))
    assert abs(local.x - 0.0) < 1e-9
    assert abs(local.y - (-1.0)) < 1e-9


def test_pose2d_local_to_world_round_trip():
    pose = Pose2d(Point2d(2.0, -1.0), Rotation2d(math.pi / 6))
    world = Point2d(5.0, 3.0)
    assert abs(pose.local_to_world(pose.world_to_local(world)).x - world.x) < 1e-9
    assert abs(pose.local_to_world(pose.world_to_local(world)).y - world.y) < 1e-9


def test_pose2d_ros_round_trip():
    pose = Pose2d(Point2d(1.0, -2.0), Rotation2d(math.pi / 4))
    restored = Pose2d.from_ros(pose.to_ros())
    assert abs(restored.point.x - pose.point.x) < 1e-6
    assert abs(restored.point.y - pose.point.y) < 1e-6
    assert abs(restored.rotation.angle - pose.rotation.angle) < 1e-6
