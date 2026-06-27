#!/usr/bin/env python3
"""Capture a single message from a ROS 2 topic to a file.

Output file format:
    # ros2_type: <message_type>
    <YAML message content>

Example:
    # ros2_type: geometry_msgs/msg/Twist
    linear:
      x: 1.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
"""

import sys
from pathlib import Path

from common import run


def main() -> None:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <topic> <output>")
        sys.exit(1)

    topic, output = sys.argv[1], sys.argv[2]

    ros_type = run("ros2", "topic", "type", topic, capture_output=True)
    echo_output = run("ros2", "topic", "echo", "--once", "--full-length", topic, capture_output=True)

    with Path(output).open("w") as f:
        f.write(f"# ros2_type: {ros_type.strip()}\n")
        f.write(echo_output)


if __name__ == "__main__":
    main()
