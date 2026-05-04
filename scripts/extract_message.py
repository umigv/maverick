#!/usr/bin/env python3
import sys

from common import run


def main() -> None:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <topic> <output>")
        sys.exit(1)

    topic, output = sys.argv[1], sys.argv[2]

    ros_type = run("ros2", "topic", "type", topic, capture_output=True)
    echo_output = run("ros2", "topic", "echo", "--once", "--full-length", topic, capture_output=True)

    with open(output, "w") as f:
        f.write(f"# ros2_type: {ros_type.strip()}\n")
        f.write(echo_output)


if __name__ == "__main__":
    main()
