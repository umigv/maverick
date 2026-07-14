#!/usr/bin/env python3
import argparse
import os
import shutil
from pathlib import Path

from common import ROOT, die, discover_targets, files_in, filter_targets, info, run


def lint_python(targets: list[Path], all_targets: list[Path]) -> None:
    if not files_in(targets, "*.py"):
        return

    target_strs = [str(t) for t in targets]

    info("Ruff format (check)")
    run("ruff", "format", "--check", *target_strs)

    info("Ruff lint")
    run("ruff", "check", *target_strs)

    mypy_dirs = [t for t in targets if any(p.name != "setup.py" for p in files_in([t], "*.py"))]
    info(f"mypy ({len(mypy_dirs)} targets)")
    if mypy_dirs:
        mypypath = ":".join(str(ROOT / t) for t in all_targets)
        run("mypy", *[str(d) for d in mypy_dirs], env={**os.environ, "MYPYPATH": mypypath})


def lint_cpp(targets: list[Path]) -> None:
    cpp = files_in(targets, "*.cpp", "*.hpp", "*.h", "*.cc")
    if not cpp:
        return

    info("clang-format (check)")
    run("clang-format", "--dry-run", "--Werror", *[str(f.relative_to(ROOT)) for f in cpp])


def lint_rviz(targets: list[Path]) -> None:
    rviz = files_in(targets, "*.rviz")
    if not rviz:
        return

    info("RViz configs (ARV Config marker)")
    for f in rviz:
        if "# ARV Config" not in f.read_text():
            die(
                f"{f.relative_to(ROOT)}: missing '# ARV Config' marker. Revert if config was accidentally saved in RViz"
                ", or re-add the marker comment if the change was intentional"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run all linters")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--only", nargs="+", metavar="PKG")
    group.add_argument("--ignore", nargs="+", metavar="PKG")
    args = parser.parse_args()

    missing = [t for t in ("ruff", "mypy", "clang-format") if not shutil.which(t)]
    if missing:
        die(f"missing tools: {', '.join(missing)}. Run just setup")

    all_targets = discover_targets()
    targets = filter_targets(all_targets, only=args.only, ignore=args.ignore)

    lint_python(targets, all_targets)
    lint_cpp(targets)
    lint_rviz(targets)

    info("All checks passed!")


if __name__ == "__main__":
    main()
