#!/usr/bin/env python3
import argparse
import shutil
from pathlib import Path

from common import ROOT, die, files_in, info, resolve_packages, run


def format_python(pkg_dirs: list[Path]) -> None:
    pkg_strs = [str(d) for d in pkg_dirs]

    info("Ruff lint (fix)")
    run("ruff", "check", "--fix", "--exit-zero", *pkg_strs)

    info("Ruff format")
    run("ruff", "format", *pkg_strs)


def format_cpp(pkg_dirs: list[Path]) -> None:
    cpp = files_in(pkg_dirs, "*.cpp", "*.hpp", "*.h", "*.cc")
    info("clang-format")
    run("clang-format", "-i", *[str(f.relative_to(ROOT)) for f in cpp])


def main() -> None:
    parser = argparse.ArgumentParser(description="Auto-format all source files")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--only", nargs="+", metavar="PKG")
    group.add_argument("--ignore", nargs="+", metavar="PKG")
    args = parser.parse_args()

    missing = [t for t in ("ruff", "clang-format") if not shutil.which(t)]
    if missing:
        die(f"missing tools: {', '.join(missing)}. Run just setup")

    pkg_dirs, _ = resolve_packages(args.only, args.ignore)

    if files_in(pkg_dirs, "*.py"):
        format_python(pkg_dirs)

    if files_in(pkg_dirs, "*.cpp", "*.hpp", "*.h", "*.cc"):
        format_cpp(pkg_dirs)


if __name__ == "__main__":
    main()
