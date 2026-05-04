#!/usr/bin/env python3
import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

BUILD_TYPES = {"python": "ament_python", "cpp": "ament_cmake"}


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a ROS 2 package")
    parser.add_argument("package_name")
    parser.add_argument("--type", choices=["python", "cpp"], default="python")
    args = parser.parse_args()

    pkg_name = args.package_name

    if not re.match(r"^[a-z][a-z0-9_]*$", pkg_name):
        print(f"ERROR: Invalid package name '{pkg_name}'", file=sys.stderr)
        print(
            "Package names must start with a lowercase letter and contain only lowercase letters, numbers, and underscores.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"==> Creating ROS 2 {args.type} package: {pkg_name}")

    subprocess.run(
        ["ros2", "pkg", "create", "--build-type", BUILD_TYPES[args.type], "--license", "Apache-2.0", pkg_name],
        check=True,
    )

    pkg_dir = Path(pkg_name)
    (pkg_dir / "LICENSE").unlink(missing_ok=True)
    shutil.rmtree(pkg_dir / "test", ignore_errors=True)

    print(f"==> Package '{pkg_name}' created successfully")


if __name__ == "__main__":
    main()
