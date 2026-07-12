from collections.abc import Callable, Iterator
from contextlib import contextmanager

import rclpy
from rclpy.executors import Executor, ExternalShutdownException
from rclpy.node import Node


@contextmanager
def managed_node[NodeT: Node](factory: Callable[[], NodeT]) -> Iterator[NodeT]:
    """Manage the node lifetime: construct on enter, destroy_node on exit."""
    node = factory()
    try:
        yield node
    finally:
        node.destroy_node()


def run_node[NodeT: Node](factory: Callable[[], NodeT], *, executor: Executor | None = None) -> None:
    """Spin a node until shutdown. Use as the whole body of a package's console_scripts `main`.

    Pass `executor` to spin on something other than the default global single-threaded executor.
    """
    try:
        with rclpy.init(), managed_node(factory) as node:
            rclpy.spin(node, executor)
    except KeyboardInterrupt, ExternalShutdownException:
        pass
