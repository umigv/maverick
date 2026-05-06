#!/usr/bin/env python3
import re
import sys
from pathlib import Path

from common import run


def main() -> None:
    if len(sys.argv) not in (3, 4):
        print(f"Usage: {sys.argv[0]} <topic> <input> [rate]")
        sys.exit(1)

    topic = sys.argv[1]
    input_file = sys.argv[2]
    rate = sys.argv[3] if len(sys.argv) > 3 else "once"

    lines = Path(input_file).read_text().splitlines(keepends=True)

    ros_type = None
    for line in lines:
        m = re.match(r"^# ros2_type:\s*(.+)", line)
        if m:
            ros_type = m.group(1).strip()
            break
    if not ros_type:
        print(f"Error: could not find '# ros2_type: ...' header in {input_file}", file=sys.stderr)
        sys.exit(1)

    payload = "".join(
        line
        for i, line in enumerate(lines)
        if not (i == 0 and re.match(r"^# ros2_type:", line)) and line.rstrip("\n") != "---"
    )

    if rate == "once":
        run("ros2", "topic", "pub", "--once", topic, ros_type, payload)
    else:
        run("ros2", "topic", "pub", "-r", rate, topic, ros_type, payload)


if __name__ == "__main__":
    main()
