#!/usr/bin/env python3
import argparse
import re
import shutil
import sys

from common import ROOT, die, run

BUILD_TYPES = {"python": "ament_python", "cpp": "ament_cmake"}


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a ROS 2 package")
    parser.add_argument("dir", help="Destination directory for the package (e.g. src/hardware)")
    parser.add_argument("package_name")
    parser.add_argument("--type", choices=["python", "cpp"], default="python")
    args = parser.parse_args()

    pkg_name = args.package_name
    dest = ROOT / args.dir

    if not re.match(r"^[a-z][a-z0-9_]*$", pkg_name):
        die(
            f"Invalid package name '{pkg_name}'. "
            "Must start with a lowercase letter and contain only lowercase letters, numbers, and underscores."
        )

    if not str(dest).startswith(str(ROOT / "src")):
        die(f"Destination must be under src/, got: {args.dir}")

    if not dest.exists():
        response = input(f"Directory {args.dir} does not exist. Create it? [y/N]: ").strip().lower()
        if response != "y":
            print("Aborted.")
            sys.exit(0)
        dest.mkdir(parents=True)

    print(f"==> Creating ROS 2 {args.type} package: {pkg_name}")
    run("ros2", "pkg", "create", "--build-type", BUILD_TYPES[args.type], "--license", "Apache-2.0", pkg_name, cwd=dest)

    pkg_dir = dest / pkg_name
    (pkg_dir / "LICENSE").unlink(missing_ok=True)
    shutil.rmtree(pkg_dir / "test", ignore_errors=True)

    print(f"==> Package '{pkg_name}' created successfully at {args.dir}/{pkg_name}")


if __name__ == "__main__":
    main()
