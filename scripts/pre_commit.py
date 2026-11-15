#!/usr/bin/env python3
from pathlib import Path

from common import discover_targets, info, run, target_from_file


def get_staged_files() -> list[Path]:
    out = run("git", "diff", "--cached", "--name-only", "--diff-filter=ACMR", capture_output=True)
    return [Path(line) for line in out.splitlines() if line]


def main() -> None:
    staged = get_staged_files()
    if not staged:
        return

    targets = discover_targets()

    affected: set[str] = set()
    for f in staged:
        target = target_from_file(f, targets)
        if target is not None:
            affected.add(target.name)

    if not affected:
        return

    info(f"pre-commit: linting {', '.join(sorted(affected))}")
    run("just", "lint", "--only", *sorted(affected))


if __name__ == "__main__":
    main()
