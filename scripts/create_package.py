#!/usr/bin/env python3
import argparse
import re
import shutil
from pathlib import Path

from common import ROOT, die, run

BUILD_TYPES = {"python": "ament_python", "cpp": "ament_cmake"}

# ros2 pkg create bakes these linters into the template package.xml, but we lint with ruff + mypy + clang-format
# (see pyproject.toml) and ship no ament lint stubs, so strip them from every new package. The ament_python template
# emits the first three; the ament_cmake template emits the last two (plus a CMake block, see strip_lint_cmake_block).
LINT_TEST_DEPENDS = ("ament_copyright", "ament_flake8", "ament_pep257", "ament_lint_auto", "ament_lint_common")


def strip_lint_test_depends(package_xml: Path) -> None:
    lines = package_xml.read_text().splitlines(keepends=True)
    kept = [line for line in lines if not any(f"<test_depend>{dep}</test_depend>" in line for dep in LINT_TEST_DEPENDS)]
    package_xml.write_text(re.sub(r"\n{3,}", "\n\n", "".join(kept)))


def strip_lint_cmake_block(cmakelists: Path) -> None:
    # ament_cmake's template adds an `if(BUILD_TESTING) ... endif()` block that runs ament_lint_auto; drop it
    # so the package builds without the linter deps we just removed from package.xml.
    out: list[str] = []
    skipping = False
    for line in cmakelists.read_text().splitlines():
        if line.strip() == "if(BUILD_TESTING)":
            skipping = True
        elif skipping:
            skipping = line.strip() != "endif()"
        else:
            out.append(line)
    cmakelists.write_text(re.sub(r"\n{3,}", "\n\n", "\n".join(out) + "\n"))


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
            return
        dest.mkdir(parents=True)

    print(f"==> Creating ROS 2 {args.type} package: {pkg_name}")
    run("ros2", "pkg", "create", "--build-type", BUILD_TYPES[args.type], "--license", "Apache-2.0", pkg_name, cwd=dest)

    pkg_dir = dest / pkg_name
    (pkg_dir / "LICENSE").unlink(missing_ok=True)
    shutil.rmtree(pkg_dir / "test", ignore_errors=True)
    strip_lint_test_depends(pkg_dir / "package.xml")
    if args.type == "cpp":
        strip_lint_cmake_block(pkg_dir / "CMakeLists.txt")

    print(f"==> Package '{pkg_name}' created successfully at {args.dir}/{pkg_name}")


if __name__ == "__main__":
    main()
