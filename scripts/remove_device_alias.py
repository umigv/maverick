#!/usr/bin/env python3
import argparse
from pathlib import Path

from common import die, info, run


def main() -> None:
    parser = argparse.ArgumentParser(description="Remove a udev device alias.")
    parser.add_argument("alias", help="alias name, e.g. imu")
    args = parser.parse_args()

    rules_file = Path(f"/etc/udev/rules.d/99-{args.alias}.rules")

    if not rules_file.exists():
        die(f"No rule found for alias '{args.alias}' (expected {rules_file}).")

    info(f"Removing rule at {rules_file}:\n{rules_file.read_text().strip()}")

    run("sudo", "rm", str(rules_file))
    run("sudo", "udevadm", "control", "--reload-rules")
    run("sudo", "udevadm", "trigger")

    info(f"Udev rule removed. /dev/{args.alias} will no longer be created.")


if __name__ == "__main__":
    main()
