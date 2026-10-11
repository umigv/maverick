import dataclasses
import math
from itertools import pairwise
from typing import Any

import numpy as np
import pytest
from geometry_msgs.msg import Twist
from path_tracking.controllers.mppi.controller import MPPIController, wrap_angle
from path_tracking.path_tracking_config import PathTrackingConfig
from rclpy.logging import get_logger
from utils.geometry import Path2d, Point2d, Pose2d, Rotation2d

CONTROL_PERIOD_S = 0.01
CONFIG = dataclasses.replace(PathTrackingConfig.mppi, seed=0)
MAX_DV_MPS = CONFIG.max_linear_accel_mps2 * CONTROL_PERIOD_S


def make_controller(**overrides: Any) -> MPPIController:
    return MPPIController(dataclasses.replace(CONFIG, **overrides), CONTROL_PERIOD_S, get_logger("test_mppi"))


def straight_path(length_m: float = 10.0, heading_rad: float = 0.0) -> Path2d:
    direction = Point2d(x=math.cos(heading_rad), y=math.sin(heading_rad))
    return Path2d([direction * (length_m * i / 10) for i in range(11)])


def pose(x: float, y: float, yaw: float) -> Pose2d:
    return Pose2d(point=Point2d(x=x, y=y), rotation=Rotation2d(yaw))


def run_closed_loop(controller: MPPIController, robot: Pose2d, steps: int) -> tuple[Pose2d, list[Twist]]:
    """Integrate the unicycle at the control period with each returned command, passing back the true speed."""
    commands = []
    speed_mps = float(controller.last_command[0])  # the robot moves at the last published command
    for _ in range(steps):
        cmd = controller.compute_command(robot, speed_mps)
        if cmd is None:
            break
        commands.append(cmd)
        speed_mps = cmd.linear.x
        heading = robot.rotation.angle
        robot = pose(
            robot.point.x + cmd.linear.x * math.cos(heading) * CONTROL_PERIOD_S,
            robot.point.y + cmd.linear.x * math.sin(heading) * CONTROL_PERIOD_S,
            heading + cmd.angular.z * CONTROL_PERIOD_S,
        )
    return robot, commands


def test_no_path_returns_none() -> None:
    assert make_controller().compute_command(pose(0.0, 0.0, 0.0), 0.0) is None


def test_set_path_resets_progress_and_nominal() -> None:
    controller = make_controller()
    controller.set_path(straight_path())
    run_closed_loop(controller, pose(0.0, 0.0, 0.0), 100)
    assert controller.projection_index > 0

    controller.set_path(straight_path())

    assert controller.projection_index == 0
    np.testing.assert_array_equal(controller.nominal, np.tile(controller.last_command, (CONFIG.horizon_steps, 1)))


def test_straight_path_moves_forward() -> None:
    controller = make_controller()
    controller.set_path(straight_path())

    robot, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 150)

    assert all(math.isfinite(c.linear.x) and math.isfinite(c.angular.z) for c in commands)
    assert commands[-1].linear.x > 1.0
    assert robot.point.x > 1.0
    assert abs(robot.point.y) < 0.2


@pytest.mark.parametrize("offset_m", [1.0, -1.0])
def test_offset_path_steers_toward_path(offset_m: float) -> None:
    controller = make_controller()
    controller.set_path(Path2d([Point2d(x=0.0, y=offset_m), Point2d(x=10.0, y=offset_m)]))

    _, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 30)

    # Path to the left (+y) requires turning counter-clockwise (+omega)
    assert math.copysign(1.0, commands[-1].angular.z) == math.copysign(1.0, offset_m)
    robot, _ = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 300)
    assert abs(robot.point.y - offset_m) < 0.3


def test_curved_path_turns_left() -> None:
    controller = make_controller()
    radius = 3.0
    controller.set_path(
        Path2d([Point2d(x=radius * math.sin(a), y=radius * (1 - math.cos(a))) for a in np.linspace(0, math.pi, 30)])
    )

    _, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 100)

    assert commands[-1].angular.z > 0.0


def test_goal_returns_one_zero_command_then_none() -> None:
    controller = make_controller()
    controller.set_path(straight_path(length_m=1.0))

    cmd = controller.compute_command(pose(0.9, 0.0, 0.0), 0.0)

    assert cmd == Twist()
    assert controller.compute_command(pose(0.9, 0.0, 0.0), 0.0) is None
    np.testing.assert_array_equal(controller.nominal, 0.0)


def test_closed_loop_reaches_goal() -> None:
    controller = make_controller()
    controller.set_path(straight_path(length_m=5.0))

    robot, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 1000)

    assert commands[-1] == Twist()
    assert controller.compute_command(robot, 0.0) is None
    assert robot.point.distance(Point2d(x=5.0, y=0.0)) < CONFIG.goal_tolerance_m + 0.05
    # The speed taper ends where the robot stops (goal_tolerance_m before the end), so the final zero command is a small
    # step rather than a stop from speed
    assert commands[-2].linear.x < 0.3


@pytest.mark.parametrize(
    "overrides",
    [{}, {"reuse_noise": False, "linear_noise_std_mps": 0.4, "angular_noise_std_radps": 0.8}],
)
def test_commands_respect_limits(overrides: dict[str, Any]) -> None:
    controller = make_controller(**overrides)
    controller.set_path(Path2d([Point2d(x=0.0, y=0.0), Point2d(x=4.0, y=0.0), Point2d(x=4.0, y=4.0)]))

    _, commands = run_closed_loop(controller, pose(0.0, -1.0, math.pi / 2), 1500)

    # The goal stop is a single zero command by design and is exempt from the rate limits
    assert commands.pop() == Twist()
    half_track = CONFIG.track_width_m / 2
    max_dv = CONFIG.max_linear_accel_mps2 * CONTROL_PERIOD_S + 1e-9
    max_domega = CONFIG.max_angular_accel_radps2 * CONTROL_PERIOD_S + 1e-9
    previous = Twist()
    for cmd in commands:
        v, omega = cmd.linear.x, cmd.angular.z
        assert math.isfinite(v)
        assert math.isfinite(omega)
        assert 0.0 <= v <= CONFIG.max_linear_speed_mps + 1e-9
        assert abs(omega) <= CONFIG.max_angular_speed_radps + 1e-9
        assert abs(v - previous.linear.x) <= max_dv
        assert abs(omega - previous.angular.z) <= max_domega
        assert abs(v - omega * half_track) <= CONFIG.max_wheel_speed_mps + 1e-9
        assert abs(v + omega * half_track) <= CONFIG.max_wheel_speed_mps + 1e-9
        previous = cmd


def test_wrap_angle() -> None:
    np.testing.assert_allclose(wrap_angle(math.pi + 0.1), -math.pi + 0.1)
    np.testing.assert_allclose(wrap_angle(-math.pi - 0.1), math.pi - 0.1)
    np.testing.assert_allclose(wrap_angle(0.5), 0.5)


@pytest.mark.parametrize("yaw", [math.pi - 0.01, -math.pi + 0.01])
def test_path_heading_near_pi_does_not_spin(yaw: float) -> None:
    controller = make_controller()
    controller.set_path(straight_path(heading_rad=math.pi))

    _, commands = run_closed_loop(controller, pose(0.0, 0.0, yaw), 100)

    assert commands[-1].linear.x > 1.0
    assert all(abs(c.angular.z) < 0.5 for c in commands)


def test_fixed_seed_is_deterministic() -> None:
    results = []
    for _ in range(2):
        controller = make_controller()
        controller.set_path(straight_path())
        _, commands = run_closed_loop(controller, pose(0.0, 0.5, 0.3), 20)
        results.append([(c.linear.x, c.angular.z) for c in commands])
    assert results[0] == results[1]


@pytest.mark.parametrize(
    "overrides",
    [
        {"target_speed_mps": 0.0},
        {"num_samples": 1},
        {"horizon_steps": 0},
        {"timestep_s": -0.1},
        {"temperature": 0.0},
        {"linear_noise_std_mps": 0.0},
        {"track_width_m": 0.0},
        {"cross_track_weight": -1.0},
        {"target_speed_mps": 2.5, "max_linear_speed_mps": 2.0, "max_wheel_speed_mps": 3.0},
        {"seed": -2},
        {"speed_resync_threshold_mps": 0.0},
        {"reuse_noise": True, "num_samples": 3},
        {"reuse_noise": True, "num_samples": 11},
        {"reuse_noise": True, "noise_refresh_pairs_per_call": 0},
        {"reuse_noise": True, "num_samples": 10, "noise_refresh_pairs_per_call": 5},
    ],
)
def test_invalid_config_rejected(overrides: dict[str, Any]) -> None:
    with pytest.raises(ValueError, match="MPPIConfig"):
        dataclasses.replace(CONFIG, **overrides)


def test_reuse_settings_are_not_validated_without_reuse() -> None:
    config = dataclasses.replace(CONFIG, reuse_noise=False, num_samples=3, noise_refresh_pairs_per_call=5)
    assert config.num_samples == 3


def hairpin_path() -> Path2d:
    """3 m out along +x, a 180 degree turn of radius 0.5 m, then 3 m back along y = 1."""
    out = [Point2d(x=0.25 * i, y=0.0) for i in range(13)]
    turn = [Point2d(x=3.0 + 0.5 * math.sin(a), y=0.5 - 0.5 * math.cos(a)) for a in np.linspace(0.2, math.pi - 0.2, 15)]
    back = [Point2d(x=3.0 - 0.25 * i, y=1.0) for i in range(13)]
    return Path2d(out + turn + back)


def test_hairpin_is_followed_not_cut() -> None:
    controller = make_controller()
    path = hairpin_path()
    controller.set_path(path)

    robot = pose(0.0, 0.0, 0.0)
    max_x_m, max_cross_track_m = 0.0, 0.0
    for _ in range(2000):
        robot, commands = run_closed_loop(controller, robot, 1)
        if not commands:
            break
        max_x_m = max(max_x_m, robot.point.x)
        projection = path.project(robot.point)
        assert projection is not None
        max_cross_track_m = max(max_cross_track_m, path[projection].distance(robot.point))

    assert controller.path is None  # reached the goal
    # The path turns around at x = 3.5. Cutting across to the return leg skips the turn and the space between the legs,
    # which is where the obstacle the planner went around would be.
    assert max_x_m > 3.2
    assert max_cross_track_m < 0.4


def drive_to_speed() -> tuple[MPPIController, Pose2d]:
    controller = make_controller()
    controller.set_path(straight_path(length_m=20.0))
    robot, _ = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 150)
    assert controller.last_command[0] > CONFIG.speed_resync_threshold_mps + 0.5
    return controller, robot


@pytest.mark.parametrize("measured_offset_mps", [-(CONFIG.speed_resync_threshold_mps - 0.05), 1.0])
def test_speed_within_threshold_or_above_keeps_command(measured_offset_mps: float) -> None:
    controller, robot = drive_to_speed()
    previous_mps = float(controller.last_command[0])

    cmd = controller.compute_command(robot, previous_mps + measured_offset_mps)

    assert cmd is not None
    assert abs(cmd.linear.x - previous_mps) <= MAX_DV_MPS + 1e-9


def test_speed_below_threshold_resynchronizes_command() -> None:
    controller, robot = drive_to_speed()
    measured_mps = float(controller.last_command[0]) - CONFIG.speed_resync_threshold_mps - 0.2

    cmd = controller.compute_command(robot, measured_mps)

    assert cmd is not None
    assert cmd.linear.x <= measured_mps + MAX_DV_MPS + 1e-9


def test_downstream_speed_cap_bounds_command_jump() -> None:
    """The node caps linear.x during a ramp approach. MPPI must track the capped speed, not its own uncapped command."""
    controller = make_controller()
    controller.set_path(straight_path(length_m=20.0))
    robot, speed_mps, cap_mps = pose(0.0, 0.0, 0.0), 0.0, 1.0
    applied = []
    for step in range(450):
        measured_mps = speed_mps
        cmd = controller.compute_command(robot, measured_mps)
        assert cmd is not None
        # The stored command never leads the speed it was given by more than the threshold plus one rate step
        assert controller.last_command[0] - measured_mps <= CONFIG.speed_resync_threshold_mps + MAX_DV_MPS + 1e-9
        capped = 150 <= step < 300
        speed_mps = min(cmd.linear.x, cap_mps) if capped else cmd.linear.x
        applied.append(speed_mps)
        heading = robot.rotation.angle
        robot = pose(
            robot.point.x + speed_mps * math.cos(heading) * CONTROL_PERIOD_S,
            robot.point.y + speed_mps * math.sin(heading) * CONTROL_PERIOD_S,
            heading + cmd.angular.z * CONTROL_PERIOD_S,
        )

    assert max(applied[:150]) > cap_mps + CONFIG.speed_resync_threshold_mps  # the cap actually limited the robot
    jump_mps = max(abs(b - a) for a, b in pairwise(applied[290:311]))
    assert jump_mps <= CONFIG.speed_resync_threshold_mps + MAX_DV_MPS + 1e-9


def test_reused_noise_is_mirrored_and_renews_one_pair_per_call() -> None:
    controller = make_controller(reuse_noise=True)
    half = CONFIG.num_samples // 2

    first = controller.exploration_noise()
    second = controller.exploration_noise()

    np.testing.assert_allclose(first.sum(axis=0), 0.0, atol=1e-12)  # mirrored pairs: exactly zero mean
    np.testing.assert_array_equal(first[half:], -first[:half])
    np.testing.assert_array_equal(first[0], 0.0)  # the nominal sample stays unperturbed
    changed = np.flatnonzero(np.any(first != second, axis=(1, 2)))
    assert changed.tolist() == [1, half + 1]  # one sample and its mirror replaced per call

    controller = make_controller(reuse_noise=True, noise_refresh_pairs_per_call=3)
    first = controller.exploration_noise()
    changed = np.flatnonzero(np.any(first != controller.exploration_noise(), axis=(1, 2)))
    assert changed.tolist() == [1, 2, 3, half + 1, half + 2, half + 3]


def count_speed_reversals(commands: list[Twist]) -> int:
    changes = np.diff([c.linear.x for c in commands])
    signs = np.sign(changes[np.abs(changes) > 0.005])
    return int(np.sum(signs[1:] != signs[:-1]))


def test_reused_noise_reduces_command_chatter() -> None:
    results = {}
    for name, overrides in (
        ("fresh", {"reuse_noise": False, "linear_noise_std_mps": 0.4, "angular_noise_std_radps": 0.8}),
        ("reused", {}),  # the defaults: reused, halved noise
    ):
        controller = make_controller(**overrides)
        controller.set_path(Path2d([Point2d(x=0.2 * i, y=0.5) for i in range(76)]))
        _, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 500)
        results[name] = count_speed_reversals(commands[100:])

    assert results["reused"] < results["fresh"] / 3
