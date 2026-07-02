import utils.config
import utils.lifecycle
from rclpy.node import Node

from .template_python_config import TemplatePythonConfig


class TemplatePython(Node):
    def __init__(self) -> None:
        super().__init__("template_python")

        self.config = utils.config.load(self, TemplatePythonConfig)


def main() -> None:
    utils.lifecycle.run_node(TemplatePython)
