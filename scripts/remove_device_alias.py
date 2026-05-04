#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path


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

    subprocess.run(["sudo", "rm", str(rules_file)], check=True)
    subprocess.run(["sudo", "udevadm", "control", "--reload-rules"], check=True)
    subprocess.run(["sudo", "udevadm", "trigger"], check=True)

    print(f"Udev rule removed. /dev/{alias} will no longer be created.")


if __name__ == "__main__":
    main()
