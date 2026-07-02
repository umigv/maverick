#!/usr/bin/env python3
import argparse
from pathlib import Path

from common import run


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a message to a ROS 2 topic from a file.")
    parser.add_argument("topic", help="topic to publish to")
    parser.add_argument("input", help="file containing the YAML message content")
    parser.add_argument("rate", nargs="?", default="once", help="publish rate in Hz, or 'once' (default)")
    args = parser.parse_args()

    # run() exits with a logged error if the topic type can't be resolved (e.g. no node is using the topic).
    ros_type = run("ros2", "topic", "type", args.topic, capture_output=True).strip()
    payload = Path(args.input).read_text()
    rate_args = ["--once"] if args.rate == "once" else ["-r", args.rate]
    run("ros2", "topic", "pub", *rate_args, args.topic, ros_type, payload)


if __name__ == "__main__":
    main()
