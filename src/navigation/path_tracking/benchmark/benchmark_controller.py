#!/usr/bin/env python3
"""Offline compute benchmark for path tracking controllers.

Runs a controller in closed loop on a synthetic S-curve with an ideal unicycle model stepped at the control period, and
times each compute_command call. Not a real-time test: results depend on the machine and its load.

Usage: pixi run python3 src/navigation/path_tracking/benchmark/benchmark_controller.py --algorithm mppi [--samples K]
"""

import argparse
import dataclasses
import json
import math
import platform
import resource
import statistics
import time

import numpy as np
from path_tracking.controllers.controller import Controller
from path_tracking.controllers.differential_drive.controller import DifferentialDriveController
from path_tracking.controllers.mppi.controller import MPPIController
from path_tracking.path_tracking_config import PathTrackingConfig
from rclpy.logging import get_logger
from utils.geometry import Path2d, Point2d, Pose2d, Rotation2d


def s_curve(length_m: float, amplitude_m: float, num_points: int) -> Path2d:
    s = np.linspace(0.0, 1.0, num_points)
    return Path2d([Point2d(x=float(length_m * t), y=float(amplitude_m * math.sin(2 * math.pi * t))) for t in s])


def percentile(sorted_values: list[float], q: float) -> float:
    return sorted_values[min(len(sorted_values) - 1, round(q * (len(sorted_values) - 1)))]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--algorithm", choices=["mppi", "differential_drive"], default="mppi")
    parser.add_argument("--samples", type=int, help="override mppi.num_samples")
    parser.add_argument("--horizon-steps", type=int, help="override mppi.horizon_steps")
    parser.add_argument("--timestep", type=float, help="override mppi.timestep_s")
    parser.add_argument("--calls", type=int, default=1000, help="measured calls (loops over the path as needed)")
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    period_s = PathTrackingConfig.control_period_s
    logger = get_logger("benchmark_controller")
    controller: Controller
    if args.algorithm == "mppi":
        overrides = {
            "num_samples": args.samples,
            "horizon_steps": args.horizon_steps,
            "timestep_s": args.timestep,
            "seed": args.seed,
        }
        mppi_config = dataclasses.replace(
            PathTrackingConfig.mppi, **{k: v for k, v in overrides.items() if v is not None}
        )
        controller = MPPIController(mppi_config, period_s, logger)
        config_summary = dataclasses.asdict(mppi_config)
    else:
        controller = DifferentialDriveController(PathTrackingConfig.differential_drive, logger)
        config_summary = dataclasses.asdict(PathTrackingConfig.differential_drive)

    path = s_curve(length_m=16.0, amplitude_m=1.5, num_points=200)
    latencies_s: list[float] = []
    cpu_s = 0.0
    speeds: list[float] = []
    laps = 0
    while len(latencies_s) < args.calls:
        controller.set_path(path)
        x, y, yaw, speed_mps = 0.0, 0.0, 0.0, 0.0
        laps += 1
        while len(latencies_s) < args.calls:
            pose = Pose2d(point=Point2d(x=x, y=y), rotation=Rotation2d(yaw))
            wall_start, cpu_start = time.perf_counter(), time.process_time()
            cmd = controller.compute_command(pose, speed_mps)
            wall, cpu = time.perf_counter() - wall_start, time.process_time() - cpu_start
            if cmd is None:
                break
            if args.warmup > 0:
                args.warmup -= 1
            else:
                latencies_s.append(wall)
                cpu_s += cpu
                speeds.append(cmd.linear.x)
            speed_mps = cmd.linear.x  # the ideal plant tracks the command exactly
            x += cmd.linear.x * math.cos(yaw) * period_s
            y += cmd.linear.x * math.sin(yaw) * period_s
            yaw += cmd.angular.z * period_s

    ms = sorted(t * 1e3 for t in latencies_s)
    mean_ms = statistics.mean(ms)
    period_ms = period_s * 1e3
    result = {
        "algorithm": args.algorithm,
        "calls": len(ms),
        "laps": laps,
        "mean_ms": mean_ms,
        "p50_ms": percentile(ms, 0.5),
        "p95_ms": percentile(ms, 0.95),
        "max_ms": ms[-1],
        "calls_per_s": 1e3 / mean_ms,
        "control_period_ms": period_ms,
        "deadline_miss_pct": 100.0 * sum(t > period_ms for t in ms) / len(ms),
        "real_time_factor": period_ms / mean_ms,
        "cpu_util_during_calls_pct": 100.0 * cpu_s / sum(latencies_s),
        "core_share_at_control_rate_pct": 100.0 * mean_ms / period_ms,
        "max_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        "mean_speed_mps": statistics.mean(speeds),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "machine": platform.machine(),
        "config": config_summary,
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
