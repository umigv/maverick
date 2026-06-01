#!/usr/bin/env python3
"""
Live-plot motor current and velocity for both ODrive controllers.

Subscribes to /odrive_driver/debug (Float32MultiArray) and displays four
real-time plots: left/right Iq setpoint vs measured (A), and left/right
velocity setpoint vs estimate (rps).

Requires odrive_driver to be running with debug: true in its config.

Usage:
    just plot-odrive [-- --window 1000]
"""

import argparse
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

_FIELDS = ["l_iq_sp", "l_iq_meas", "l_vel_sp", "l_vel_est", "r_iq_sp", "r_iq_meas", "r_vel_sp", "r_vel_est"]


class MotorSignalPlotter(Node):
    def __init__(self, window: int):
        super().__init__("odrive_plot")

        self._data = {k: deque(maxlen=window) for k in _FIELDS}

        self._fig, axes = plt.subplots(2, 2, sharex=True, figsize=(12, 7))
        self._fig.suptitle("ODrive diagnostics")
        self._axes = axes

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

        (l_iq_sp,) = ax_l_iq.plot([], [], label="setpoint", color="tab:blue")
        (l_iq_meas,) = ax_l_iq.plot([], [], label="measured", color="tab:orange")
        (r_iq_sp,) = ax_r_iq.plot([], [], label="setpoint", color="tab:blue")
        (r_iq_meas,) = ax_r_iq.plot([], [], label="measured", color="tab:orange")
        (l_vel_sp,) = ax_l_vel.plot([], [], label="setpoint", color="tab:blue")
        (l_vel_est,) = ax_l_vel.plot([], [], label="estimate", color="tab:orange")
        (r_vel_sp,) = ax_r_vel.plot([], [], label="setpoint", color="tab:blue")
        (r_vel_est,) = ax_r_vel.plot([], [], label="estimate", color="tab:orange")
        for ax in axes.flat:
            ax.legend(loc="upper right")

        self._lines = (l_iq_sp, l_iq_meas, r_iq_sp, r_iq_meas, l_vel_sp, l_vel_est, r_vel_sp, r_vel_est)

        self.create_subscription(Float32MultiArray, "/odrive_driver/debug", self._on_debug, 10)

    def _on_debug(self, msg: Float32MultiArray) -> None:
        for key, val in zip(self._data.keys(), msg.data, strict=True):
            self._data[key].append(val)

    def update(self, _) -> tuple:
        rclpy.spin_once(self, timeout_sec=0)
        xs = range(len(self._data["l_iq_sp"]))
        for line, key in zip(self._lines, _FIELDS, strict=True):
            line.set_data(xs, self._data[key])
        for ax in self._axes.flat:
            ax.relim()
            ax.autoscale_view()
        return self._lines

    def show(self) -> None:
        self._ani = animation.FuncAnimation(self._fig, self.update, interval=100, blit=False)
        plt.tight_layout()
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--window", type=int, default=500, metavar="N", help="number of samples to display (default: 500)"
    )
    args = parser.parse_args()

    rclpy.init()
    plotter = MotorSignalPlotter(window=args.window)
    plotter.show()
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
