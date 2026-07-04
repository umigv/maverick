#!/usr/bin/env python3
import argparse
from pathlib import Path

from common import run


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture a single message from a topic to a file.")
    parser.add_argument("topic", help="topic to capture from")
    parser.add_argument("output", help="file to write the YAML message to")
    args = parser.parse_args()

    echo_output = run("ros2", "topic", "echo", "--once", "--full-length", args.topic, capture_output=True)

    # ros2 topic echo delimits messages with a trailing '---'; drop it so the file is a single clean YAML document.
    payload = "".join(line for line in echo_output.splitlines(keepends=True) if line.rstrip("\n") != "---")
    Path(args.output).write_text(payload)


if __name__ == "__main__":
    main()
