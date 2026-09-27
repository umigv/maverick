#!/usr/bin/env python3
import argparse
import re
from pathlib import Path

from common import ROOT, die, info, run

TEMPLATES = {"python": ROOT / "src/template/template_python", "cpp": ROOT / "src/template/template_cpp"}


def pascal_case(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_"))


def substitute(text: str, template_name: str, pkg_name: str) -> str:
    return text.replace(template_name, pkg_name).replace(pascal_case(template_name), pascal_case(pkg_name))


def copy_template(template: Path, pkg_dir: Path, pkg_name: str) -> None:
    for src_path in sorted(template.rglob("*")):
        rel = src_path.relative_to(template)
        if "__pycache__" in rel.parts:
            continue
        out_path = pkg_dir / Path(*(substitute(part, template.name, pkg_name) for part in rel.parts))
        if src_path.is_dir():
            out_path.mkdir(parents=True, exist_ok=True)
        else:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(substitute(src_path.read_text(), template.name, pkg_name))


def git_config(key: str, default: str) -> str:
    return run("git", "config", "--default", default, "--get", key, capture_output=True).strip()


def set_maintainer(pkg_dir: Path) -> None:
    name = git_config("user.name", "TODO")
    email = git_config("user.email", "todo@todo.todo")

    for filename in ("package.xml", "setup.py"):
        path = pkg_dir / filename
        if path.exists():
            path.write_text(
                path.read_text()
                .replace("Hardworking ARV Member", name)
                .replace("hardworking_arv_member@umich.edu", email)
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a ROS 2 package from the template packages in src/template")
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

    pkg_dir = dest / pkg_name
    if pkg_dir.exists():
        die(f"Package directory already exists: {args.dir}/{pkg_name}")

    if not dest.exists():
        response = input(f"Directory {args.dir} does not exist. Create it? [y/N]: ").strip().lower()
        if response != "y":
            info("Aborted.")
            return
        dest.mkdir(parents=True)

    info(f"Creating ROS 2 {args.type} package: {pkg_name}")
    copy_template(TEMPLATES[args.type], pkg_dir, pkg_name)
    set_maintainer(pkg_dir)

    info(f"Package '{pkg_name}' created successfully at {args.dir}/{pkg_name}")


if __name__ == "__main__":
    main()
