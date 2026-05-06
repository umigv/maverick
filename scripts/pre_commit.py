#!/usr/bin/env python3
"""Pre-commit hook that lints only packages touched by staged files."""

import subprocess
import sys
from pathlib import Path

from common import ROOT, discover_packages


def get_staged_files() -> list[Path]:
    result = subprocess.run(
        ["git", "diff", "--cached", "--name-only", "--diff-filter=ACMR"],
        capture_output=True,
        text=True,
        cwd=ROOT,
    )
    return [Path(line) for line in result.stdout.splitlines() if line]


def file_to_package(filepath: Path, pkg_dirs: list[Path]) -> str | None:
    if filepath.parts[0] == "scripts":
        return "scripts"

    for pkg_dir in sorted(pkg_dirs, key=lambda p: len(p.parts), reverse=True):
        if filepath == pkg_dir or pkg_dir in filepath.parents:
            return pkg_dir.name

    return None


def main() -> int:
    staged = get_staged_files()
    if not staged:
        return 0

    pkg_dirs = discover_packages()

    affected: set[str] = set()
    for f in staged:
        pkg = file_to_package(f, pkg_dirs)
        if pkg is not None:
            affected.add(pkg)

    if not affected:
        return 0

    print(f"pre-commit: linting {', '.join(sorted(affected))}")
    result = subprocess.run(
        ["just", "lint", "--only", *sorted(affected)],
        cwd=ROOT,
    )
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
