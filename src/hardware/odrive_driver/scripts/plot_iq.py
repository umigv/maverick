#!/usr/bin/env python3
"""
Read odrive_driver log output from stdin and live-plot Iq and velocity signals.

Usage:
    ros2 run odrive_driver odrive_driver 2>&1 | python3 scripts/plot_iq.py
    # or replay a saved log:
    python3 scripts/plot_iq.py < odrive.log
"""

import re
import sys
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt

PATTERN = re.compile(
    r"left Iq_setpoint=([+-]?\d+\.\d+)\s+Iq_measured=([+-]?\d+\.\d+)\s+"
    r"vel_setpoint=([+-]?\d+\.\d+)\s+vel_estimate=([+-]?\d+\.\d+)"
    r".*?right Iq_setpoint=([+-]?\d+\.\d+)\s+Iq_measured=([+-]?\d+\.\d+)\s+"
    r"vel_setpoint=([+-]?\d+\.\d+)\s+vel_estimate=([+-]?\d+\.\d+)"
)

WINDOW = 500

data = {
    k: deque(maxlen=WINDOW)
    for k in [
        "l_iq_sp",
        "l_iq_meas",
        "l_vel_sp",
        "l_vel_est",
        "r_iq_sp",
        "r_iq_meas",
        "r_vel_sp",
        "r_vel_est",
    ]
}

fig, axes = plt.subplots(2, 2, sharex=True, figsize=(12, 7))
fig.suptitle("ODrive diagnostics")

(ax_l_iq, ax_r_iq), (ax_l_vel, ax_r_vel) = axes

for ax, title in [
    (ax_l_iq, "Left Iq (A)"),
    (ax_r_iq, "Right Iq (A)"),
    (ax_l_vel, "Left velocity (rps)"),
    (ax_r_vel, "Right velocity (rps)"),
]:
    ax.set_title(title)
    ax.grid(True)

for ax in (ax_l_vel, ax_r_vel):
    ax.set_xlabel("sample")

(line_l_iq_sp,) = ax_l_iq.plot([], [], label="setpoint", color="tab:blue")
(line_l_iq_meas,) = ax_l_iq.plot([], [], label="measured", color="tab:orange")
(line_r_iq_sp,) = ax_r_iq.plot([], [], label="setpoint", color="tab:blue")
(line_r_iq_meas,) = ax_r_iq.plot([], [], label="measured", color="tab:orange")
(line_l_vel_sp,) = ax_l_vel.plot([], [], label="setpoint", color="tab:blue")
(line_l_vel_est,) = ax_l_vel.plot([], [], label="estimate", color="tab:orange")
(line_r_vel_sp,) = ax_r_vel.plot([], [], label="setpoint", color="tab:blue")
(line_r_vel_est,) = ax_r_vel.plot([], [], label="estimate", color="tab:orange")

for ax in axes.flat:
    ax.legend(loc="upper right")

all_lines = (
    line_l_iq_sp,
    line_l_iq_meas,
    line_r_iq_sp,
    line_r_iq_meas,
    line_l_vel_sp,
    line_l_vel_est,
    line_r_vel_sp,
    line_r_vel_est,
)


def _read_stdin():
    for raw in sys.stdin:
        m = PATTERN.search(raw)
        if m:
            vals = [float(g) for g in m.groups()]
            for key, val in zip(data, vals, strict=True):
                data[key].append(val)


def _update(_frame):
    _read_stdin()
    xs = range(len(data["l_iq_sp"]))
    pairs = [
        (line_l_iq_sp, data["l_iq_sp"]),
        (line_l_iq_meas, data["l_iq_meas"]),
        (line_r_iq_sp, data["r_iq_sp"]),
        (line_r_iq_meas, data["r_iq_meas"]),
        (line_l_vel_sp, data["l_vel_sp"]),
        (line_l_vel_est, data["l_vel_est"]),
        (line_r_vel_sp, data["r_vel_sp"]),
        (line_r_vel_est, data["r_vel_est"]),
    ]
    for line, buf in pairs:
        line.set_data(xs, buf)
    for ax in axes.flat:
        ax.relim()
        ax.autoscale_view()
    return all_lines


ani = animation.FuncAnimation(fig, _update, interval=100, blit=False)

plt.tight_layout()
plt.show()
