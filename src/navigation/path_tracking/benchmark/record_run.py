#!/usr/bin/env python3
"""Record path tracking signals during a robot or simulation run, for analyze_run.py.

Records nav_cmd_vel, odom/local (pose and measured speed), smoothed_path, goal, the capture-to-receipt age of
occupancy_grid/raw, and once per second the CPU and memory of the path_tracking process. Each message is written to
--output as one JSON line, so stopping with Ctrl-C keeps everything recorded so far. --output must not exist yet, so two
runs are never mixed in one file.

Usage:
    pixi run python3 src/navigation/path_tracking/benchmark/record_run.py --output /tmp/run.jsonl
"""

import argparse
import json
import math
import os
import time
from pathlib import Path
from typing import IO, Any

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from utils.geometry import Pose2d


def find_pid(node_name: str) -> int | None:
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if f"__node:={node_name}".encode() in (proc / "cmdline").read_bytes().split(b"\0"):
                return int(proc.name)
        except OSError:
            continue
    return None


def process_usage(pid: int) -> tuple[float, float]:
    """Return (CPU seconds used so far, resident memory in MB) of a process."""
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    cpu_s = (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
    rss_mb = math.nan
    for line in Path(f"/proc/{pid}/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            rss_mb = int(line.split()[1]) / 1024
    return cpu_s, rss_mb


class RunRecorder(Node):
    def __init__(self, output: IO[str]) -> None:
        super().__init__("run_recorder")
        self.output = output
        self.create_subscription(TwistStamped, "nav_cmd_vel", self.on_command, 100)
        self.create_subscription(Odometry, "odom/local", self.on_odom, 100)
        self.create_subscription(PathMsg, "smoothed_path", self.on_path, 10)
        self.create_subscription(PointStamped, "goal", self.on_goal, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid/raw", self.on_grid, 10)
        self.create_timer(1.0, self.sample_process)
        self.last_cpu: tuple[float, float] | None = None

    def write(self, kind: str, **fields: Any) -> None:
        self.output.write(json.dumps({"t": time.monotonic(), "type": kind, **fields}) + "\n")

    def on_command(self, msg: TwistStamped) -> None:
        self.write("cmd", v=msg.twist.linear.x, w=msg.twist.angular.z)

    def on_odom(self, msg: Odometry) -> None:
        pose = Pose2d.from_ros(msg.pose.pose)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.write("odom", x=pose.point.x, y=pose.point.y, yaw=pose.rotation.angle, speed=speed)

    def on_path(self, msg: PathMsg) -> None:
        self.write("path", points=[[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def on_goal(self, msg: PointStamped) -> None:
        self.write("goal", x=msg.point.x, y=msg.point.y)

    def on_grid(self, msg: OccupancyGrid) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0:
            self.write("grid", age_s=self.get_clock().now().nanoseconds * 1e-9 - stamp)

    def sample_process(self) -> None:
        pid = find_pid("path_tracking")
        if pid is None:
            return
        try:
            cpu_s, rss_mb = process_usage(pid)
        except OSError:
            return
        now = time.monotonic()
        if self.last_cpu is not None:
            self.write("process", cpu_pct=100.0 * (cpu_s - self.last_cpu[1]) / (now - self.last_cpu[0]), rss_mb=rss_mb)
        self.last_cpu = (now, cpu_s)
        self.output.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with args.output.open("x") as output:
        rclpy.init()
        recorder = RunRecorder(output)
        try:
            rclpy.spin(recorder)
        except KeyboardInterrupt, ExternalShutdownException:
            pass
        finally:
            recorder.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
