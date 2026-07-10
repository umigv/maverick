#!/usr/bin/env python3
from pathlib import Path

from common import discover_packages, file_to_package, info, run


def get_staged_files() -> list[Path]:
    out = run("git", "diff", "--cached", "--name-only", "--diff-filter=ACMR", capture_output=True)
    return [Path(line) for line in out.splitlines() if line]


def main() -> None:
    staged = get_staged_files()
    if not staged:
        return

    pkg_dirs = discover_packages()

    affected: set[str] = set()
    for f in staged:
        pkg = file_to_package(f, pkg_dirs)
        if pkg is not None:
            affected.add(pkg)

    if not affected:
        return

    info(f"pre-commit: linting {', '.join(sorted(affected))}")
    run("just", "lint", "--only", *sorted(affected))


if __name__ == "__main__":
    main()
