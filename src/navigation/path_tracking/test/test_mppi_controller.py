import dataclasses
import math
from itertools import pairwise
from typing import Any

import numpy as np
import pytest
from geometry_msgs.msg import Twist
from path_tracking.controllers.mppi.config import MPPIConfig
from path_tracking.controllers.mppi.controller import MPPIController, wrap_angle
from path_tracking.path_tracking_config import PathTrackingConfig
from rclpy.logging import get_logger
from utils.geometry import Path2d, Point2d, Pose2d, Rotation2d

# Most tests drive a simulated robot in a loop: ask the controller for a command, move a perfect unicycle by that
# command for one control period, repeat (see run_closed_loop).

CONTROL_PERIOD_S = 0.01
DEFAULT = PathTrackingConfig.mppi
# The real defaults, but with a fixed seed so every run draws the same noise and the tests are repeatable
CONFIG = dataclasses.replace(DEFAULT, noise=dataclasses.replace(DEFAULT.noise, seed=0))
# The most the speed command may change in one control period
MAX_DV_MPS = CONFIG.limits.max_linear_accel_mps2 * CONTROL_PERIOD_S


def replace_config(**overrides: Any) -> MPPIConfig:
    """Return CONFIG with overrides applied. A dict changes fields of a group, e.g. noise={"seed": 1}."""
    return dataclasses.replace(
        CONFIG,
        **{
            name: dataclasses.replace(getattr(CONFIG, name), **value) if isinstance(value, dict) else value
            for name, value in overrides.items()
        },
    )


def make_controller(**overrides: Any) -> MPPIController:
    return MPPIController(replace_config(**overrides), CONTROL_PERIOD_S, get_logger("test_mppi"))


def straight_path(length_m: float = 10.0, heading_rad: float = 0.0) -> Path2d:
    """11 evenly spaced points from the origin, length_m long, pointing at heading_rad."""
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
        if cmd is None:  # no path anymore, i.e. the goal was reached
            break
        commands.append(cmd)
        speed_mps = cmd.linear.x
        # Perfect robot: it does exactly what it was told for one control period
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
    assert controller.projection_index > 0  # make sure we actually made progress before the new path

    controller.set_path(straight_path())

    # A new path starts from the beginning, and the plan restarts as "keep doing the last command"
    assert controller.projection_index == 0
    np.testing.assert_array_equal(
        controller.nominal, np.tile(controller.last_command, (CONFIG.sampling.horizon_steps, 1))
    )


def test_straight_path_moves_forward() -> None:
    controller = make_controller()
    controller.set_path(straight_path())

    robot, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 150)

    assert all(math.isfinite(c.linear.x) and math.isfinite(c.angular.z) for c in commands)
    assert commands[-1].linear.x > 1.0  # sped up
    assert robot.point.x > 1.0  # went somewhere
    assert abs(robot.point.y) < 0.2  # and stayed on the line


@pytest.mark.parametrize("offset_m", [1.0, -1.0])
def test_offset_path_steers_toward_path(offset_m: float) -> None:
    # The path is a straight line offset_m to the side of the robot
    controller = make_controller()
    controller.set_path(Path2d([Point2d(x=0.0, y=offset_m), Point2d(x=10.0, y=offset_m)]))

    _, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 30)

    # Path to the left (+y) requires turning counter-clockwise (+omega)
    assert math.copysign(1.0, commands[-1].angular.z) == math.copysign(1.0, offset_m)
    # Given more time, the robot ends up on the line
    robot, _ = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 300)
    assert abs(robot.point.y - offset_m) < 0.3


def test_curved_path_turns_left() -> None:
    # Half circle bending left (counter-clockwise), starting at the robot and heading along +x
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

    # 0.1 m from the end of the path, which is inside goal_tolerance_m
    cmd = controller.compute_command(pose(0.9, 0.0, 0.0), 0.0)

    # One stop command, then the controller forgets the path and the plan is cleared
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


def test_commands_respect_limits() -> None:
    # An L-shaped path with a sharp 90 degree corner, starting 1 m off the path and facing the wrong way, so the
    # controller has every reason to push against its limits
    controller = make_controller()
    controller.set_path(Path2d([Point2d(x=0.0, y=0.0), Point2d(x=4.0, y=0.0), Point2d(x=4.0, y=4.0)]))

    _, commands = run_closed_loop(controller, pose(0.0, -1.0, math.pi / 2), 1500)

    # The goal stop is a single zero command by design and is exempt from the rate limits
    assert commands.pop() == Twist()
    half_track = CONFIG.limits.track_width_m / 2
    # Largest allowed change in v and omega between two commands
    max_dv = CONFIG.limits.max_linear_accel_mps2 * CONTROL_PERIOD_S + 1e-9
    max_domega = CONFIG.limits.max_angular_accel_radps2 * CONTROL_PERIOD_S + 1e-9
    previous = Twist()
    for cmd in commands:
        v, omega = cmd.linear.x, cmd.angular.z
        assert math.isfinite(v)
        assert math.isfinite(omega)
        # Speed limits
        assert 0.0 <= v <= CONFIG.limits.max_linear_speed_mps + 1e-9
        assert abs(omega) <= CONFIG.limits.max_angular_speed_radps + 1e-9
        # Acceleration limits
        assert abs(v - previous.linear.x) <= max_dv
        assert abs(omega - previous.angular.z) <= max_domega
        # Each wheel's speed (v -/+ omega * half_track) stays under the wheel limit
        assert abs(v - omega * half_track) <= CONFIG.limits.max_wheel_speed_mps + 1e-9
        assert abs(v + omega * half_track) <= CONFIG.limits.max_wheel_speed_mps + 1e-9
        previous = cmd


def test_wrap_angle() -> None:
    np.testing.assert_allclose(wrap_angle(math.pi + 0.1), -math.pi + 0.1)
    np.testing.assert_allclose(wrap_angle(-math.pi - 0.1), math.pi - 0.1)
    np.testing.assert_allclose(wrap_angle(0.5), 0.5)


@pytest.mark.parametrize("yaw", [math.pi - 0.01, -math.pi + 0.01])
def test_path_heading_near_pi_does_not_spin(yaw: float) -> None:
    # Path and robot both point roughly along -x, where yaw flips between +pi and -pi. If the heading error weren't
    # wrapped, the controller would see an error of almost 2 pi and spin around.
    controller = make_controller()
    controller.set_path(straight_path(heading_rad=math.pi))

    _, commands = run_closed_loop(controller, pose(0.0, 0.0, yaw), 100)

    assert commands[-1].linear.x > 1.0
    assert all(abs(c.angular.z) < 0.5 for c in commands)


def test_fixed_seed_is_deterministic() -> None:
    # Two new controllers with the same seed and the same start must produce exactly the same commands
    results = []
    for _ in range(2):
        controller = make_controller()
        controller.set_path(straight_path())
        _, commands = run_closed_loop(controller, pose(0.0, 0.5, 0.3), 20)
        results.append([(c.linear.x, c.angular.z) for c in commands])
    assert results[0] == results[1]


# Each case breaks exactly one rule in MPPIConfig (or one of its groups)
@pytest.mark.parametrize(
    "overrides",
    [
        {"target_speed_mps": 0.0},
        {"sampling": {"num_samples": 1}},
        {"sampling": {"horizon_steps": 0}},
        {"sampling": {"timestep_s": -0.1}},
        {"sampling": {"temperature": 0.0}},
        {"noise": {"linear_noise_std_mps": 0.0}},
        {"limits": {"track_width_m": 0.0}},
        {"cost": {"cross_track_weight": -1.0}},
        # Target speed faster than the speed limit (the wheel limit is raised so only the speed limit is broken)
        {"target_speed_mps": 2.5, "limits": {"max_linear_speed_mps": 2.0, "max_wheel_speed_mps": 3.0}},
        {"noise": {"seed": -2}},
        {"speed_resync_threshold_mps": 0.0},
        {"sampling": {"num_samples": 11}},  # odd: samples come in mirrored pairs
    ],
)
def test_invalid_config_rejected(overrides: dict[str, Any]) -> None:
    with pytest.raises(ValueError, match="MPPIConfig"):
        replace_config(**overrides)


def hairpin_path() -> Path2d:
    """3 m out along +x, a 180 degree turn of radius 0.5 m, then 3 m back along y = 1."""
    out = [Point2d(x=0.25 * i, y=0.0) for i in range(13)]
    turn = [Point2d(x=3.0 + 0.5 * math.sin(a), y=0.5 - 0.5 * math.cos(a)) for a in np.linspace(0.2, math.pi - 0.2, 15)]
    back = [Point2d(x=3.0 - 0.25 * i, y=1.0) for i in range(13)]
    return Path2d(out + turn + back)


def test_hairpin_is_followed_not_cut() -> None:
    # The two legs of the hairpin are only 1 m apart, so it's tempting for rollouts to cut straight across to the return
    # leg. PROJECTION_ARC_MARGIN_M is what stops that.
    controller = make_controller()
    path = hairpin_path()
    controller.set_path(path)

    robot = pose(0.0, 0.0, 0.0)
    # Farthest the robot got along +x, and the farthest it ever was from the path
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
    """Return a controller and robot that are cruising down a long straight path, well above the resync threshold."""
    controller = make_controller()
    controller.set_path(straight_path(length_m=20.0))
    robot, _ = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 150)
    assert controller.last_command[0] > CONFIG.speed_resync_threshold_mps + 0.5
    return controller, robot


# The measured speed is a bit slower than the last command (but within the threshold), or faster than it
@pytest.mark.parametrize("measured_offset_mps", [-(CONFIG.speed_resync_threshold_mps - 0.05), 1.0])
def test_speed_within_threshold_or_above_keeps_command(measured_offset_mps: float) -> None:
    controller, robot = drive_to_speed()
    previous_mps = float(controller.last_command[0])

    cmd = controller.compute_command(robot, previous_mps + measured_offset_mps)

    # No resync: the new command is still within one acceleration step of the old one
    assert cmd is not None
    assert abs(cmd.linear.x - previous_mps) <= MAX_DV_MPS + 1e-9


def test_speed_below_threshold_resynchronizes_command() -> None:
    controller, robot = drive_to_speed()
    # The robot is going clearly slower than we told it to (e.g. something downstream capped it)
    measured_mps = float(controller.last_command[0]) - CONFIG.speed_resync_threshold_mps - 0.2

    cmd = controller.compute_command(robot, measured_mps)

    # Resync: the new command starts from the measured speed, not the old command
    assert cmd is not None
    assert cmd.linear.x <= measured_mps + MAX_DV_MPS + 1e-9


def test_downstream_speed_cap_bounds_command_jump() -> None:
    """The node caps linear.x during a ramp approach. MPPI must track the capped speed, not its own uncapped command."""
    controller = make_controller()
    controller.set_path(straight_path(length_m=20.0))
    robot, speed_mps, cap_mps = pose(0.0, 0.0, 0.0), 0.0, 1.0
    applied = []  # the speed the robot actually drove at each step, after the cap
    for step in range(450):
        measured_mps = speed_mps
        cmd = controller.compute_command(robot, measured_mps)
        assert cmd is not None
        # The stored command never leads the speed it was given by more than the threshold plus one rate step
        assert controller.last_command[0] - measured_mps <= CONFIG.speed_resync_threshold_mps + MAX_DV_MPS + 1e-9
        # Steps 150-299 play the ramp approach: the robot can't go faster than cap_mps whatever we command
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
    # Right when the cap is lifted (around step 300), the speed must ramp up, not jump to the old uncapped command
    jump_mps = max(abs(b - a) for a, b in pairwise(applied[290:311]))
    assert jump_mps <= CONFIG.speed_resync_threshold_mps + MAX_DV_MPS + 1e-9


def test_reused_noise_is_mirrored_and_renews_one_pair_per_call() -> None:
    controller = make_controller()
    half = CONFIG.sampling.num_samples // 2

    # Two calls in a row, each (K, T, 2)
    first = controller.exploration_noise()
    second = controller.exploration_noise()

    np.testing.assert_allclose(first.sum(axis=0), 0.0, atol=1e-12)  # mirrored pairs: exactly zero mean
    np.testing.assert_array_equal(first[half:], -first[:half])
    np.testing.assert_array_equal(first[0], 0.0)  # the nominal sample stays unperturbed
    # Indices of the samples that differ between the two calls
    changed = np.flatnonzero(np.any(first != second, axis=(1, 2)))
    assert changed.tolist() == [1, half + 1]  # one sample and its mirror replaced per call


def count_speed_reversals(commands: list[Twist]) -> int:
    """Count how often the speed command flips between speeding up and slowing down (ignoring tiny changes)."""
    changes = np.diff([c.linear.x for c in commands])
    signs = np.sign(changes[np.abs(changes) > 0.005])
    return int(np.sum(signs[1:] != signs[:-1]))


def test_speed_command_does_not_chatter() -> None:
    # A straight path 0.5 m to the side. Once the robot has settled onto it, the speed command should be smooth rather
    # than flipping up and down every few steps.
    controller = make_controller()
    controller.set_path(Path2d([Point2d(x=0.2 * i, y=0.5) for i in range(76)]))

    _, commands = run_closed_loop(controller, pose(0.0, 0.0, 0.0), 500)

    # Skip the first second while the robot is still getting onto the path
    assert count_speed_reversals(commands[100:]) < 10
