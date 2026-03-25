from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass, field

from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Path

from .math import clamp


@dataclass
class Rotation2d:
    """A 2D rotation represented as a yaw angle in radians.

    The angle is automatically wrapped to [-π, π] on construction. cos and sin
    are cached to avoid redundant trigonometry when rotating multiple points by
    the same rotation.

    Attributes:
        angle: Yaw angle in radians, wrapped to [-π, π].
    """

    angle: float
    _cos: float = field(init=False, repr=False, compare=False)
    _sin: float = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.angle = (self.angle + math.pi) % (2 * math.pi) - math.pi
        self._cos = math.cos(self.angle)
        self._sin = math.sin(self.angle)

    @property
    def cos(self) -> float:
        """Cached cosine of the angle."""
        return self._cos

    @property
    def sin(self) -> float:
        """Cached sine of the angle."""
        return self._sin

    def __neg__(self) -> Rotation2d:
        """Return the inverse rotation."""
        return Rotation2d(-self.angle)

    def __add__(self, other: Rotation2d) -> Rotation2d:
        """Compose two rotations by adding their angles."""
        return Rotation2d(self.angle + other.angle)

    def __sub__(self, other: Rotation2d) -> Rotation2d:
        """Return the relative rotation from other to self."""
        return Rotation2d(self.angle - other.angle)

    def __mul__(self, scalar: float) -> Rotation2d:
        """Scale this rotation's angle by a scalar."""
        return Rotation2d(self.angle * scalar)

    def __rmul__(self, scalar: float) -> Rotation2d:
        """Scale this rotation's angle by a scalar (scalar * rotation)."""
        return Rotation2d(self.angle * scalar)

    def __truediv__(self, scalar: float) -> Rotation2d:
        """Divide this rotation's angle by a scalar."""
        return Rotation2d(self.angle / scalar)

    def to_ros(self) -> Quaternion:
        """Convert to a pure-yaw ROS quaternion.

        Returns:
            Quaternion with x=0, y=0, z=sin(angle/2), w=cos(angle/2).
        """
        half = self.angle / 2.0
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    @classmethod
    def from_vector(cls, vector: Point2d) -> Rotation2d:
        """Construct from a 2D direction vector.

        Args:
            vector: Direction vector. The angle is computed as atan2(y, x).

        Returns:
            Rotation2d with angle equal to the heading of vector.
        """
        return cls(math.atan2(vector.y, vector.x))

    @classmethod
    def from_ros(cls, q: Quaternion) -> Rotation2d:
        """Construct from a ROS quaternion by extracting the yaw component.

        Args:
            q: ROS quaternion message.

        Returns:
            Rotation2d with angle equal to the yaw of q.
        """
        return cls(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


@dataclass
class Point2d:
    """A 2D point with arithmetic operators and ROS interop.

    Attributes:
        x: X component in meters.
        y: Y component in meters.
    """

    x: float
    y: float

    def __neg__(self) -> Point2d:
        """Negate both components."""
        return Point2d(x=-self.x, y=-self.y)

    def __add__(self, other: Point2d) -> Point2d:
        """Add two points component-wise."""
        return Point2d(x=self.x + other.x, y=self.y + other.y)

    def __sub__(self, other: Point2d) -> Point2d:
        """Subtract two points component-wise."""
        return Point2d(x=self.x - other.x, y=self.y - other.y)

    def __mul__(self, scalar: float) -> Point2d:
        """Scale both components by a scalar."""
        return Point2d(x=self.x * scalar, y=self.y * scalar)

    def __rmul__(self, scalar: float) -> Point2d:
        """Scale both components by a scalar (scalar * point)."""
        return Point2d(x=self.x * scalar, y=self.y * scalar)

    def __truediv__(self, scalar: float) -> Point2d:
        """Divide both components by a scalar."""
        return Point2d(x=self.x / scalar, y=self.y / scalar)

    def rotate_by(self, rotation: Rotation2d) -> Point2d:
        """Rotate this point about the origin by a rotation.

        Args:
            rotation: Rotation to apply.

        Returns:
            Rotated point.
        """
        return Point2d(
            x=rotation.cos * self.x - rotation.sin * self.y,
            y=rotation.sin * self.x + rotation.cos * self.y,
        )

    def mag(self) -> float:
        """Return the Euclidean distance from the origin.

        Returns:
            Magnitude of this point as a vector.
        """
        return math.hypot(self.x, self.y)

    def distance(self, other: Point2d) -> float:
        """Return the Euclidean distance to another point.

        Args:
            other: The other point.

        Returns:
            Distance in meters.
        """
        return math.hypot(self.x - other.x, self.y - other.y)

    def dot(self, other: Point2d) -> float:
        """Return the dot product of this point with another, treating both as vectors.

        Args:
            other: The other vector.

        Returns:
            Scalar dot product.
        """
        return self.x * other.x + self.y * other.y

    def cross(self, other: Point2d) -> float:
        """Return the 2D cross product (scalar z-component of the 3D cross product).

        Args:
            other: The other vector.

        Returns:
            self.x * other.y - self.y * other.x. Positive when other is to the left of self.
        """
        return self.x * other.y - self.y * other.x

    def lerp(self, other: Point2d, t: float) -> Point2d:
        """Linearly interpolate toward another point.

        Args:
            other: Target point.
            t: Interpolation parameter. t=0 returns self, t=1 returns other. Not clamped.

        Returns:
            self + (other - self) * t.
        """
        return self + (other - self) * t

    def to_ros(self) -> Point:
        """Convert to a ROS Point with z=0.

        Returns:
            geometry_msgs/Point with x, y copied and z=0.
        """
        return Point(x=self.x, y=self.y, z=0.0)

    @classmethod
    def from_ros(cls, point: Point) -> Point2d:
        """Construct from a ROS Point, discarding z.

        Args:
            point: ROS Point message.

        Returns:
            Point2d with x and y copied from point.
        """
        return cls(x=point.x, y=point.y)


@dataclass
class Pose2d:
    """A 2D pose consisting of a position and a rotation.

    Attributes:
        point: Position in meters.
        rotation: Heading as a Rotation2d.
    """

    point: Point2d
    rotation: Rotation2d

    def world_to_local(self, world_point: Point2d) -> Point2d:
        """Transform a world-frame point into this pose's local frame.

        Args:
            world_point: Point in the world frame.

        Returns:
            Point expressed in this pose's local frame.
        """
        return (world_point - self.point).rotate_by(-self.rotation)

    def local_to_world(self, local_point: Point2d) -> Point2d:
        """Transform a local-frame point back into the world frame.

        Args:
            local_point: Point in this pose's local frame.

        Returns:
            Point expressed in the world frame.
        """
        return local_point.rotate_by(self.rotation) + self.point

    def to_ros(self) -> Pose:
        """Convert to a ROS Pose.

        Returns:
            geometry_msgs/Pose with position from point and orientation from rotation.
        """
        return Pose(
            position=self.point.to_ros(),
            orientation=self.rotation.to_ros(),
        )

    @classmethod
    def from_ros(cls, pose: Pose) -> Pose2d:
        """Construct from a ROS Pose.

        Args:
            pose: ROS Pose message.

        Returns:
            Pose2d with position and rotation extracted from pose.
        """
        return cls(
            point=Point2d.from_ros(pose.position),
            rotation=Rotation2d.from_ros(pose.orientation),
        )


class Path2d:
    """An ordered sequence of 2D waypoints with ROS interop and common traversal operations.

    All segments must be non-zero length (no duplicate consecutive points).
    """

    def __init__(self, points: list[Point2d]) -> None:
        """Construct a Path2d from a list of waypoints.

        Args:
            points: Ordered waypoints. Must contain at least 2 points with no consecutive duplicates.

        Raises:
            ValueError: If fewer than 2 points are given or any consecutive pair is identical.
        """
        if len(points) < 2:
            raise ValueError(f"Path2d requires at least 2 points, got {len(points)}")
        for i in range(len(points) - 1):
            if points[i].distance(points[i + 1]) == 0.0:
                raise ValueError(f"Path2d requires non-zero segment lengths (duplicate point at index {i})")
        self._points = points

    def __len__(self) -> int:
        """Return the number of waypoints."""
        return len(self._points)

    def __bool__(self) -> bool:
        """Return True if the path has any waypoints."""
        return bool(self._points)

    def __iter__(self) -> Iterator[Point2d]:
        """Iterate over the waypoints in order."""
        return iter(self._points)

    def __getitem__(self, index: int | float) -> Point2d:
        """Return the waypoint at an integer index or interpolate at a fractional index.

        Integer indices follow standard Python list semantics, including negative indexing.
        Float indices use the integer part as the segment and the fractional part as the
        interpolation parameter within that segment, clamped to [0, len-1].

        Args:
            index: Integer waypoint index, or float fractional path index.

        Returns:
            Waypoint at the given index, or interpolated point for float indices.
        """
        if isinstance(index, float):
            i = int(index)
            if i < 0:
                return self._points[0]
            if i >= len(self._points) - 1:
                return self._points[-1]
            return self._points[i].lerp(self._points[i + 1], index - i)
        return self._points[index]

    def project(self, point: Point2d, start_index: int = 0) -> float | None:
        """Return the fractional path index of the closest point on the path to `point`.

        Searches forward from start_index. Prefers later segments on ties to commit forward at
        vertices. Returns None if start_index is at or past the last waypoint.

        Args:
            point: Query point to project onto the path.
            start_index: Segment index to begin searching from.

        Returns:
            Fractional path index of the closest projection, or None if start_index is out of range.
        """
        if start_index >= len(self._points) - 1:
            return None

        best_projection_index: float = start_index
        best_distance = math.inf

        for i in range(start_index, len(self._points) - 1):
            start, end = self._points[i], self._points[i + 1]
            segment = end - start
            t = clamp((point - start).dot(segment) / segment.dot(segment), min=0.0, max=1.0)
            projection = start.lerp(end, t)
            distance = projection.distance(point)

            if distance <= best_distance:
                best_projection_index = i + t
                best_distance = distance

        return best_projection_index

    def advance(self, start_index: float, distance: float) -> Point2d:
        """Walk forward along the path by `distance` meters from a fractional index.

        Args:
            start_index: Fractional path index to begin walking from.
            distance: Distance in meters to walk forward.

        Returns:
            The point `distance` meters ahead of `start_index`, or the last waypoint if the
            path runs out before `distance` is consumed.
        """
        i = int(start_index)
        t = start_index - i
        remaining = distance

        while i < len(self._points) - 1:
            start, end = self._points[i], self._points[i + 1]
            length = start.distance(end)
            rest_of_segment = length * (1.0 - t)

            if rest_of_segment >= remaining:
                return start.lerp(end, t + remaining / length)

            remaining -= rest_of_segment
            i += 1
            t = 0.0

        return self._points[-1]

    @classmethod
    def from_ros(cls, path: Path) -> Path2d:
        """Construct from a ROS Path message, discarding z and pose orientations.

        Args:
            path: ROS nav_msgs/Path message.

        Returns:
            Path2d with one waypoint per pose in path.poses.

        Raises:
            ValueError: If the path has fewer than 2 poses or contains consecutive duplicate points.
        """
        return cls([Point2d(x=p.pose.position.x, y=p.pose.position.y) for p in path.poses])
