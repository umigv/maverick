#!/usr/bin/env python3
import argparse
import shutil

from common import ROOT, Target, die, discover_targets, files_in, filter_targets, info, run


def format_python(targets: list[Target]) -> None:
    py = files_in(targets, "*.py")
    if not py:
        return

    target_strs = [str(f.relative_to(ROOT)) for f in py]

    info("Ruff lint (fix)")
    run("ruff", "check", "--fix", "--exit-zero", *target_strs)

    info("Ruff format")
    run("ruff", "format", *target_strs)


def format_cpp(targets: list[Target]) -> None:
    cpp = files_in(targets, "*.cpp", "*.hpp", "*.h", "*.cc")
    if not cpp:
        return

    info("clang-format")
    run("clang-format", "-i", *[str(f.relative_to(ROOT)) for f in cpp])


def format_md(targets: list[Target]) -> None:
    md = files_in(targets, "*.md")
    if not md:
        return

    info("mdformat")
    run("mdformat", *[str(f.relative_to(ROOT)) for f in md])


def main() -> None:
    parser = argparse.ArgumentParser(description="Auto-format all source files")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--only", nargs="+", metavar="PKG")
    group.add_argument("--ignore", nargs="+", metavar="PKG")
    args = parser.parse_args()

    missing = [t for t in ("ruff", "clang-format", "mdformat") if not shutil.which(t)]
    if missing:
        die(f"missing tools: {', '.join(missing)}. Run just setup")

    targets = filter_targets(discover_targets(), only=args.only, ignore=args.ignore)

    format_python(targets)
    format_cpp(targets)
    format_md(targets)


if __name__ == "__main__":
    main()
