#!/usr/bin/env python3
"""Run repeated headless simulation trials of one path tracking configuration and compute tracking metrics.

Each trial launches sim_trial.launch.py, republishes a fixed goal (so the planner replans as obstacles are discovered),
records topics until the robot stops near the goal or a timeout, then shuts the launch down (SIGINT, escalating to
SIGTERM/SIGKILL only if needed). Results are written as JSON to --output, which should be outside the repository.

A trial is valid only if no planned path crosses an obstacle or lane line cell of the course map and the ground truth
robot position never enters one (the simulator has no collision model, so these failures are otherwise silent).

Usage:
    pixi run python3 src/navigation/path_tracking/benchmark/sim_compare.py --algorithm mppi --output /tmp/mppi_trials
"""

import argparse
import contextlib
import json
import math
import os
import signal
import statistics
import subprocess
import time
from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from path_tracking.path_tracking_config import PathTrackingConfig
from rclpy.node import Node
from std_msgs.msg import Header
from utils.geometry import Path2d, Point2d, Pose2d, Rotation2d

LAUNCH_FILE = Path(__file__).resolve().parent / "sim_trial.launch.py"
CONTROL_PERIOD_S = PathTrackingConfig.control_period_s
# Command intervals longer than this are idle pauses (controller returned None, e.g. after a goal stop), not misses
IDLE_GAP_S = 0.25


@dataclass
class Recording:
    commands: list[tuple[float, float, float]] = field(default_factory=list)  # (t, v, omega)
    local: list[tuple[float, float, float, float]] = field(default_factory=list)  # (t, x, y, yaw)
    truth: list[tuple[float, float, float, float]] = field(default_factory=list)
    paths: list[tuple[float, Path2d]] = field(default_factory=list)


class Recorder(Node):
    def __init__(self, goal: Point2d, goal_frame: str) -> None:
        super().__init__("sim_compare_recorder")
        self.data = Recording()
        self.goal = goal
        self.goal_frame = goal_frame
        self.goal_publisher = self.create_publisher(PointStamped, "goal", 10)
        self.create_subscription(TwistStamped, "nav_cmd_vel", self.on_command, 100)
        self.create_subscription(Odometry, "odom/local", self.on_local, 100)
        self.create_subscription(Odometry, "odom/ground_truth", self.on_truth, 100)
        self.create_subscription(PathMsg, "smoothed_path", self.on_path, 10)

    def publish_goal(self) -> None:
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.goal_frame)
        self.goal_publisher.publish(PointStamped(header=header, point=Point(x=self.goal.x, y=self.goal.y)))

    def on_command(self, msg: TwistStamped) -> None:
        self.data.commands.append((time.monotonic(), msg.twist.linear.x, msg.twist.angular.z))

    def on_local(self, msg: Odometry) -> None:
        pose = Pose2d.from_ros(msg.pose.pose)
        self.data.local.append((time.monotonic(), pose.point.x, pose.point.y, pose.rotation.angle))

    def on_truth(self, msg: Odometry) -> None:
        pose = Pose2d.from_ros(msg.pose.pose)
        self.data.truth.append((time.monotonic(), pose.point.x, pose.point.y, pose.rotation.angle))

    def on_path(self, msg: PathMsg) -> None:
        with contextlib.suppress(ValueError):
            self.data.paths.append((time.monotonic(), Path2d.from_ros(msg)))


def find_pid(node_name: str, process_group: int | None = None, ros_domain_id: str | None = None) -> int | None:
    """Find a running ROS node process by name, optionally only within one process group or one ROS domain."""
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if f"__node:={node_name}".encode() not in (proc / "cmdline").read_bytes().split(b"\0"):
                continue
            pid = int(proc.name)
            if process_group is not None and os.getpgid(pid) != process_group:
                continue
            if ros_domain_id is not None and process_ros_domain_id(proc) != ros_domain_id:
                continue
        except OSError:  # the process exited, or its environment is not readable
            continue
        return pid
    return None


def process_ros_domain_id(proc: Path) -> str:
    for entry in (proc / "environ").read_bytes().split(b"\0"):
        if entry.startswith(b"ROS_DOMAIN_ID="):
            return entry.split(b"=", 1)[1].decode() or "0"
    return "0"


def process_group_alive(process_group: int) -> bool:
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return False
    return True


def cpu_seconds(pid: int) -> float:
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def rss_mb(pid: int) -> float:
    for line in Path(f"/proc/{pid}/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / 1024
    return math.nan


def stats(values: list[float]) -> dict[str, float]:
    if not values:
        return {}
    ordered = sorted(values)
    return {
        "mean": statistics.mean(ordered),
        "rms": math.sqrt(statistics.mean(v * v for v in ordered)),
        "p95": ordered[round(0.95 * (len(ordered) - 1))],
        "max": ordered[-1],
    }


def path_errors(
    samples: list[tuple[float, float, float, float]], paths: list[tuple[float, Path2d]]
) -> tuple[list[float], list[float]]:
    """Cross-track distance and absolute heading error of each sample to the most recent smoothed path."""
    cross_track, heading = [], []
    path_index = -1
    for t, x, y, yaw in samples:
        while path_index + 1 < len(paths) and paths[path_index + 1][0] <= t:
            path_index += 1
        if path_index < 0:
            continue
        path = paths[path_index][1]
        point = Point2d(x=x, y=y)
        projection = path.project(point)
        if projection is None:
            continue
        segment = min(int(projection), len(path) - 2)
        cross_track.append(path[projection].distance(point))
        heading.append(abs((Rotation2d(yaw) - Rotation2d.from_vector(path[segment + 1] - path[segment])).angle))
    return cross_track, heading


class CourseMap:
    """Occupied cells (obstacles and lane lines) of a simulation course, in the map frame (map == odom in nav_test)."""

    def __init__(self, course: str) -> None:
        map_file = Path(get_package_share_directory("bringup")) / "courses" / course / "map.json"
        data = json.loads(map_file.read_text())
        self.resolution_m: float = data["resolution_m"]
        self.cells = {(x, y) for x, y in data["obstacles"] + data.get("lane_lines", [])}

    def occupied(self, x: float, y: float) -> bool:
        return (math.floor(x / self.resolution_m), math.floor(y / self.resolution_m)) in self.cells

    def crosses(self, path: Path2d) -> bool:
        step_m = self.resolution_m / 2
        for start, end in pairwise(path):
            steps = math.ceil(start.distance(end) / step_m)
            points = (start.lerp(end, i / steps) for i in range(steps + 1))
            if any(self.occupied(point.x, point.y) for point in points):
                return True
        return False


def summarize(
    data: Recording, goal: Point2d, start: float, end: float, completed: bool, course_map: CourseMap
) -> dict[str, object]:
    def window[T: tuple[float, ...]](rows: list[T]) -> list[T]:
        return [r for r in rows if start <= r[0] <= end]

    commands, local, truth = window(data.commands), window(data.local), window(data.truth)
    cte_local, heading_local = path_errors(local, data.paths)
    cte_truth, _ = path_errors(truth, data.paths)
    distance = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in pairwise(truth))
    command_pairs = list(pairwise(commands))
    all_intervals = [b[0] - a[0] for a, b in command_pairs]
    intervals = [i for i in all_intervals if i <= IDLE_GAP_S]
    idle = [i for i in all_intervals if i > IDLE_GAP_S]
    stops = sum(1 for a, b in command_pairs if (a[1], a[2]) != (0.0, 0.0) and (b[1], b[2]) == (0.0, 0.0))
    dv = [abs(b[1] - a[1]) / max(b[0] - a[0], 1e-6) for a, b in command_pairs]
    domega = [abs(b[2] - a[2]) / max(b[0] - a[0], 1e-6) for a, b in command_pairs]
    omega_sign_changes = sum(1 for a, b in command_pairs if a[2] * b[2] < 0 and abs(a[2] - b[2]) > 0.05)
    paths_crossing = sum(1 for _, path in data.paths if course_map.crosses(path))
    truth_in_obstacle = sum(1 for _, x, y, _ in truth if course_map.occupied(x, y))
    final = truth[-1] if truth else (end, math.nan, math.nan, math.nan)
    duration = end - start
    return {
        "valid": paths_crossing == 0 and truth_in_obstacle == 0,
        "paths_crossing_obstacles": paths_crossing,
        "truth_samples_in_obstacles": truth_in_obstacle,
        "completed": completed,
        "duration_s": duration,
        "final_truth_distance_to_goal_m": math.hypot(final[1] - goal.x, final[2] - goal.y),
        "distance_traveled_m": distance,
        "average_speed_mps": distance / duration if duration > 0 else math.nan,
        "commanded_speed_mps": stats([c[1] for c in commands]),
        "commanded_abs_angular_radps": stats([abs(c[2]) for c in commands]),
        "cross_track_local_m": stats(cte_local),
        "cross_track_truth_m": stats(cte_truth),
        "heading_error_local_rad": stats(heading_local),
        "linear_accel_mps2": stats(dv),
        "angular_accel_radps2": stats(domega),
        "omega_sign_changes": omega_sign_changes,
        "num_paths_received": len(data.paths),
        "num_commands": len(commands),
        "command_interval_ms": stats([i * 1e3 for i in intervals]),
        "command_gaps_over_1_5_periods_pct": 100.0
        * sum(i > 1.5 * CONTROL_PERIOD_S for i in intervals)
        / max(len(intervals), 1),
        "idle_pauses": len(idle),
        "idle_time_s": sum(idle),
        "zero_command_stops": stops,
    }


def shutdown(launch: subprocess.Popen[bytes]) -> str:
    """Stop the launch: SIGINT to ros2 launch, then SIGTERM/SIGKILL to its process group if it does not exit."""
    for sig, wait_s in ((signal.SIGINT, 30.0), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        if sig == signal.SIGINT:
            launch.send_signal(sig)
        else:
            os.killpg(launch.pid, sig)
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            # start_new_session=True makes the launch process the leader of a group holding every node it started
            if launch.poll() is not None and not process_group_alive(launch.pid):
                return sig.name
            time.sleep(0.5)
    return "FAILED"


def run_trial(args: argparse.Namespace, trial: int, recorder: Recorder, course_map: CourseMap) -> dict[str, object]:
    launch_args = [f"course:={args.course}", f"algorithm:={args.algorithm}"]
    if args.target_speed is not None:
        launch_args.append(f"target_speed_mps:={args.target_speed}")
    log_path = args.output / f"{args.label}_trial{trial}_launch.log"
    with log_path.open("wb") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", str(LAUNCH_FILE), *launch_args],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    try:
        deadline = time.monotonic() + 60.0
        while (
            find_pid("path_tracking", process_group=launch.pid) is None or not recorder.data.local
        ) and time.monotonic() < deadline:
            rclpy.spin_once(recorder, timeout_sec=0.1)
        settle_end = time.monotonic() + args.settle
        while time.monotonic() < settle_end:
            rclpy.spin_once(recorder, timeout_sec=0.1)
        pid = find_pid("path_tracking", process_group=launch.pid)
        if pid is None:
            raise RuntimeError("path_tracking did not start")
        recorder.data = Recording()
        start = time.monotonic()
        cpu_start = cpu_seconds(pid)
        peak_rss = rss_mb(pid)
        next_goal = start
        completed = False
        while time.monotonic() - start < args.timeout:
            now = time.monotonic()
            if now >= next_goal:
                recorder.publish_goal()
                next_goal = now + 1.0 / args.goal_rate
                peak_rss = max(peak_rss, rss_mb(pid))
            rclpy.spin_once(recorder, timeout_sec=0.005)
            local = recorder.data.local
            commands = recorder.data.commands
            if (
                local
                and commands
                and math.hypot(local[-1][1] - recorder.goal.x, local[-1][2] - recorder.goal.y) < args.goal_radius
                and commands[-1][1:] == (0.0, 0.0)
            ):
                completed = True
                break
        end = time.monotonic()
        cpu_pct = 100.0 * (cpu_seconds(pid) - cpu_start) / (end - start)
        result = summarize(recorder.data, recorder.goal, start, end, completed, course_map)
        result |= {"trial": trial, "path_tracking_cpu_pct": cpu_pct, "path_tracking_peak_rss_mb": peak_rss}
    finally:
        result_shutdown = shutdown(launch)
    result["shutdown_signal"] = result_shutdown
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--algorithm", required=True)
    parser.add_argument("--target-speed", type=float, help="override <algorithm>.target_speed_mps")
    parser.add_argument("--label", help="name for output files (default: algorithm[_speed])")
    parser.add_argument("--course", default="test3")
    parser.add_argument("--goal", type=float, nargs=2, default=[7.5, 1.5], metavar=("X", "Y"), help="goal in odom (m)")
    parser.add_argument("--goal-frame", default="odom")
    parser.add_argument("--goal-rate", type=float, default=1.0, help="goal republish rate (Hz) to trigger replanning")
    parser.add_argument(
        "--goal-radius", type=float, default=0.5, help="robot within this of goal + zero cmd = complete"
    )
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--settle", type=float, default=5.0, help="wait after startup before the first goal (s)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.label = args.label or args.algorithm + (f"_{args.target_speed:g}" if args.target_speed is not None else "")
    args.output.mkdir(parents=True, exist_ok=True)

    course_map = CourseMap(args.course)
    rclpy.init()
    recorder = Recorder(Point2d(x=args.goal[0], y=args.goal[1]), args.goal_frame)
    results: list[dict[str, object]] = []
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID") or "0"

    def save() -> None:
        (args.output / f"{args.label}.json").write_text(
            json.dumps({"args": vars(args) | {"output": str(args.output)}, "trials": results}, indent=2)
        )

    try:
        for trial in range(args.trials):
            # Another path_tracking on the same ROS domain would also publish nav_cmd_vel; other domains are isolated
            if find_pid("path_tracking", ros_domain_id=ros_domain_id) is not None:
                raise RuntimeError(f"a path_tracking process is already running on ROS domain {ros_domain_id}")
            result = run_trial(args, trial, recorder, course_map)
            print(json.dumps(result))
            results.append(result)
            save()  # keep finished trials if a later one fails
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
        save()


if __name__ == "__main__":
    main()
