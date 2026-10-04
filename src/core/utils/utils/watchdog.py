from collections.abc import Callable

from rclpy.node import Node
from rclpy.timer import Timer


class Watchdog:
    """A ROS 2 watchdog that repeatedly calls a function while not being petted.

    The countdown starts on construction. Call `pet()` whenever the monitored
    activity occurs to postpone expiration by a full timeout period. Without a
    pet, the callback runs every `timeout_sec` seconds.
    """

    _timer: Timer

    def __init__(self, node: Node, timeout_sec: float, callback: Callable[[], None]) -> None:
        """Construct the watchdog and start its repeating expiration timer.

        Args:
            node: ROS 2 node whose clock and executor drive the watchdog.
            timeout_sec: Timeout in seconds.
            callback: Synchronous function called with no arguments on each expiration.
        """
        self._timer = node.create_timer(timeout_sec, callback)

    def pet(self) -> None:
        """Reset the countdown to a full timeout period from now."""
        self._timer.reset()
