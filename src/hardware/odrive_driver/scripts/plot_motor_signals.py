import argparse
from collections import deque

import matplotlib.pyplot as plt
import utils.lifecycle
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

FIELDS = ["l_iq_sp", "l_iq_meas", "l_vel_sp", "l_vel_est", "r_iq_sp", "r_iq_meas", "r_vel_sp", "r_vel_est"]


class MotorSignalPlotter(Node):
    def __init__(self, window: int, frame_rate: float = 10.0):
        super().__init__("odrive_plot")

        self.data: dict[str, deque[float]] = {k: deque(maxlen=window) for k in FIELDS}

        self.fig, self.axes = plt.subplots(2, 2, sharex=True, figsize=(12, 7))
        self.fig.suptitle("ODrive diagnostics")

        (ax_l_iq, ax_r_iq), (ax_l_vel, ax_r_vel) = self.axes
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
        for ax in self.axes.flat:
            ax.legend(loc="upper right")

        self.lines = (l_iq_sp, l_iq_meas, r_iq_sp, r_iq_meas, l_vel_sp, l_vel_est, r_vel_sp, r_vel_est)

        self.create_subscription(Float32MultiArray, "odrive_driver/debug", self.callback, 100)
        self.create_timer(1.0 / frame_rate, self.render)

        plt.tight_layout()
        plt.ion()
        plt.show(block=False)

    def callback(self, msg: Float32MultiArray) -> None:
        for key, val in zip(self.data.keys(), msg.data, strict=True):
            self.data[key].append(val)

    def render(self) -> None:
        # Once the window is closed its canvas is freed, drawing on it would be a use after free in the GUI backend
        if not plt.fignum_exists(self.fig.number):
            raise SystemExit(0)

        xs = range(len(self.data["l_iq_sp"]))
        for line, key in zip(self.lines, FIELDS, strict=True):
            line.set_data(xs, self.data[key])

        for ax in self.axes.flat:
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--window", type=int, default=500, metavar="N", help="Number of samples to display (default: 500)"
    )
    parser.add_argument(
        "--frame-rate", type=float, default=10.0, metavar="HZ", help="Plot redraw rate in Hz (default: 10)"
    )
    args = parser.parse_args()

    utils.lifecycle.run_node(lambda: MotorSignalPlotter(window=args.window, frame_rate=args.frame_rate))


if __name__ == "__main__":
    main()
