#!/usr/bin/env bash
# Compute benchmark for the robot computer: records the machine, then repeats the MPPI and differential_drive
# controller benchmarks. Run from the repository root after `just build`, once idle and once with the CV stack running:
#   src/navigation/path_tracking/benchmark/robot_benchmark.sh /tmp/robot_benchmark_idle
set -euo pipefail
out=${1:?usage: robot_benchmark.sh <output directory>}
mkdir -p "$out"
benchmark=src/navigation/path_tracking/benchmark/benchmark_controller.py
{
  date --iso-8601=seconds
  uname -a
  lscpu | grep -E "Model name|^CPU\(s\)|Thread|MHz"
  nproc
  free -h
  echo "governors: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort | uniq -c | tr '\n' ' ')"
  uptime
} > "$out/machine.txt"
for i in 1 2 3; do
  pixi run python3 "$benchmark" --algorithm mppi --calls 3000 > "$out/mppi_$i.json"
done
pixi run python3 "$benchmark" --algorithm mppi --samples 250 --calls 3000 > "$out/mppi_k250.json"
pixi run python3 "$benchmark" --algorithm differential_drive --calls 3000 > "$out/differential_drive.json"
for f in "$out"/*.json; do
  python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(sys.argv[1], {k: round(d[k], 2) for k in ('mean_ms','p95_ms','max_ms','deadline_miss_pct','core_share_at_control_rate_pct')})" "$f"
done | tee "$out/summary.txt"
