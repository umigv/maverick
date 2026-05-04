#!/usr/bin/env python3
import sys
from pathlib import Path

from common import run


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <alias> (e.g. {sys.argv[0]} imu)")
        sys.exit(1)

    alias = sys.argv[1]
    rules_file = Path(f"/etc/udev/rules.d/99-{alias}.rules")

    if not rules_file.exists():
        print(f"ERROR: No rule found for alias '{alias}' (expected {rules_file}).", file=sys.stderr)
        sys.exit(1)

    print(f"Removing rule at {rules_file}:")
    print(f"  {rules_file.read_text().strip()}")

    run("sudo", "rm", str(rules_file))
    run("sudo", "udevadm", "control", "--reload-rules")
    run("sudo", "udevadm", "trigger")

    print(f"Udev rule removed. /dev/{alias} will no longer be created.")


if __name__ == "__main__":
    main()
