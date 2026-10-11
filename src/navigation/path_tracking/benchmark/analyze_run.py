#!/usr/bin/env python3
"""Summarize a run recorded with record_run.py.

Reports command timing (deadline misses), commanded speed, wheel acceleration chatter after a model of the ODrive
velocity ramp, how far the commanded speed leads the measured speed (to check mppi.speed_resync_threshold_mps),
cross-track and heading error to the latest path, planner latency (goal to path), occupancy grid age, and path_tracking
CPU and memory.

Usage:
    pixi run python3 src/navigation/path_tracking/benchmark/analyze_run.py /tmp/run.jsonl
"""

import argparse
import contextlib
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
from path_tracking.path_tracking_config import PathTrackingConfig
from utils.geometry import Path2d, Point2d, Rotation2d


def stats(values: list[float]) -> dict[str, float]:
    if not values:
        return {}
    ordered = sorted(values)
    return {
        "mean": float(np.mean(ordered)),
        "rms": math.sqrt(float(np.mean(np.square(ordered)))),
        "p95": ordered[round(0.95 * (len(ordered) - 1))],
        "max": ordered[-1],
    }


def path_errors(samples: np.ndarray, paths: list[tuple[float, Path2d]]) -> tuple[list[float], list[float]]:
    """Cross-track distance and absolute heading error of each (t, x, y, yaw) sample to the latest path at that time."""
    cross_track, heading = [], []
    path_index = -1
    for t, x, y, yaw in samples:
        while path_index + 1 < len(paths) and paths[path_index + 1][0] <= t:
            path_index += 1
        if path_index < 0:
            continue
        path = paths[path_index][1]
        point = Point2d(x=float(x), y=float(y))
        projection = path.project(point)
        if projection is None:
            continue
        segment = min(int(projection), len(path) - 2)
        cross_track.append(path[projection].distance(point))
        tangent = Rotation2d.from_vector(path[segment + 1] - path[segment])
        heading.append(abs((Rotation2d(float(yaw)) - tangent).angle))
    return cross_track, heading


def wheel_chatter(
    commands: np.ndarray, track_width_m: float, accel_limit_mps2: float, rate_hz: float = 1000.0
) -> dict[str, float]:
    """Wheel acceleration reversals and fast swing size after a per-wheel slew-rate limit (the ODrive VEL_RAMP)."""
    t = np.arange(commands[0, 0], commands[-1, 0] + 0.2, 1.0 / rate_hz)
    index = np.searchsorted(commands[:, 0], t, side="right") - 1
    v, w = commands[index, 1].copy(), commands[index, 2].copy()
    stale = t - commands[index, 0] > 0.5  # the driver zeroes commands older than its watchdog
    v[stale] = 0.0
    w[stale] = 0.0
    duration = t[-1] - t[0]
    out: dict[str, list[float]] = defaultdict(list)
    for target in (v - w * track_width_m / 2, v + w * track_width_m / 2):
        speed = np.empty_like(target)
        current = 0.0
        step = accel_limit_mps2 / rate_hz
        for i, goal in enumerate(target):
            current += min(max(goal - current, -step), step)
            speed[i] = current
        accel = np.diff(speed) * rate_hz
        for name, threshold in (("reversals_per_s", 0.5), ("large_reversals_per_s", 1.5)):
            signs = np.sign(np.where(np.abs(accel) > threshold, accel, 0.0))
            signs = signs[signs != 0]
            out[name].append(float(np.sum(signs[1:] != signs[:-1]) / duration))
        window = int(0.2 * rate_hz)
        fast = (accel - np.convolve(accel, np.ones(window) / window, mode="same"))[window:-window]
        out["fast_swing_rms_mps2"].append(float(fast.std()))
        out["time_at_ramp_limit_pct"].append(float(np.mean(np.abs(accel) > 0.99 * accel_limit_mps2) * 100))
    return {name: float(np.mean(values)) for name, values in out.items()}


def analyze(records: list[dict[str, Any]], period_s: float, track_width_m: float, accel_mps2: float) -> dict[str, Any]:
    by_type: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        by_type[record["type"]].append(record)
    commands = np.array([(r["t"], r["v"], r["w"]) for r in by_type["cmd"]])
    odom = np.array([(r["t"], r["x"], r["y"], r["yaw"], r["speed"]) for r in by_type["odom"]])
    paths = []
    for r in by_type["path"]:
        with contextlib.suppress(ValueError):  # Path2d rejects fewer than 2 points or repeated consecutive points
            paths.append((r["t"], Path2d([Point2d(x=x, y=y) for x, y in r["points"]])))
    result: dict[str, Any] = {"duration_s": records[-1]["t"] - records[0]["t"] if records else 0.0}

    if len(commands) > 1:
        intervals = np.diff(commands[:, 0])
        active = intervals[intervals <= 0.25]  # longer gaps are idle (no path), not deadline misses
        result["command_interval_ms"] = stats(list(active * 1e3))
        result["command_gaps_over_1_5_periods_pct"] = (
            float(np.mean(active > 1.5 * period_s) * 100) if len(active) else 0.0
        )
        result["commanded_speed_mps"] = stats(list(commands[:, 1]))
        result["wheel_chatter"] = wheel_chatter(commands, track_width_m, accel_mps2)
    if len(commands) > 1 and len(odom):
        # The speed passed to the controller at each call is the latest measured speed before it
        index = np.searchsorted(odom[:, 0], commands[1:, 0], side="right") - 1
        valid = index >= 0
        lead = commands[:-1, 1][valid] - odom[index[valid], 4]
        moving = (commands[:-1, 1][valid] > 0) | (odom[index[valid], 4] > 0)
        lead = lead[moving]
        if len(lead):
            threshold = PathTrackingConfig.mppi.speed_resync_threshold_mps
            result["command_lead_over_measured_speed_mps"] = {
                "p50": float(np.percentile(lead, 50)),
                "p99": float(np.percentile(lead, 99)),
                "p99_9": float(np.percentile(lead, 99.9)),
                "max": float(lead.max()),
                f"calls_above_resync_threshold_{threshold}": int(np.sum(lead > threshold)),
            }
    if paths and len(odom):
        cross_track, heading = path_errors(odom[:, :4], paths)
        result["cross_track_m"] = stats(cross_track)
        result["heading_error_rad"] = stats(heading)
    # Each path answers the most recent goal before it (a goal that yields no path has no latency)
    goal_times = np.array([g["t"] for g in by_type["goal"]])
    latencies = []
    for t, _ in paths:
        earlier = goal_times[goal_times <= t]
        if len(earlier):
            latencies.append(float(t - earlier[-1]))
    if latencies:
        result["planner_latency_s"] = stats(latencies)
    if by_type["grid"]:
        result["grid_age_s"] = stats([r["age_s"] for r in by_type["grid"]])
    if by_type["process"]:
        result["path_tracking_cpu_pct"] = stats([r["cpu_pct"] for r in by_type["process"]])
        result["path_tracking_rss_mb"] = max(r["rss_mb"] for r in by_type["process"] if not math.isnan(r["rss_mb"]))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", type=Path)
    parser.add_argument("--track-width", type=float, default=0.764, help="odrive_driver track_width_m")
    parser.add_argument("--accel-limit", type=float, default=3.0, help="odrive_driver accel_limit_mps2")
    args = parser.parse_args()
    records = [json.loads(line) for line in args.log.read_text().splitlines() if line.strip()]
    records.sort(key=lambda r: r["t"])
    summary = analyze(records, PathTrackingConfig.control_period_s, args.track_width, args.accel_limit)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
