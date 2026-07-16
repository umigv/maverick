#!/usr/bin/env python3
import argparse
import os
import shutil

from common import ROOT, Target, die, discover_targets, files_in, filter_targets, info, run


def lint_python(targets: list[Target], all_targets: list[Target]) -> None:
    py = files_in(targets, "*.py")
    if not py:
        return

    target_strs = [str(f.relative_to(ROOT)) for f in py]

    info("Ruff format (check)")
    run("ruff", "format", "--check", *target_strs)

    info("Ruff lint")
    run("ruff", "check", *target_strs)

    mypy_files = [p for p in files_in(targets, "*.py") if p.name != "setup.py"]
    info(f"mypy ({len(mypy_files)} targets)")
    if mypy_files:
        mypypath = ":".join(str(ROOT / t.path) for t in all_targets)
        run("mypy", *[str(f.relative_to(ROOT)) for f in mypy_files], env={**os.environ, "MYPYPATH": mypypath})


def lint_cpp(targets: list[Target]) -> None:
    cpp = files_in(targets, "*.cpp", "*.hpp", "*.h", "*.cc")
    if not cpp:
        return

    info("clang-format (check)")
    run("clang-format", "--dry-run", "--Werror", *[str(f.relative_to(ROOT)) for f in cpp])


def lint_rviz(targets: list[Target]) -> None:
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


def lint_md(targets: list[Target]) -> None:
    md = files_in(targets, "*.md")
    if not md:
        return

    info("mdformat")
    run("mdformat", "--check", *[str(f.relative_to(ROOT)) for f in md])


def main() -> None:
    parser = argparse.ArgumentParser(description="Run all linters")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--only", nargs="+", metavar="PKG")
    group.add_argument("--ignore", nargs="+", metavar="PKG")
    args = parser.parse_args()

    missing = [t for t in ("ruff", "mypy", "clang-format", "mdformat") if not shutil.which(t)]
    if missing:
        die(f"missing tools: {', '.join(missing)}. Run just setup")

    all_targets = discover_targets()
    targets = filter_targets(all_targets, only=args.only, ignore=args.ignore)

    lint_python(targets, all_targets)
    lint_cpp(targets)
    lint_rviz(targets)
    lint_md(targets)

    info("All checks passed!")


if __name__ == "__main__":
    main()
