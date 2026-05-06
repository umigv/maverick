#!/usr/bin/env python3
import argparse
import os
import shutil
from pathlib import Path

from common import ROOT, die, files_in, resolve_packages, run


def lint_python(pkg_dirs: list[Path], all_pkg_dirs: list[Path]) -> None:
    pkg_strs = [str(d) for d in pkg_dirs]

    print("==> Ruff format (check)")
    run("ruff", "format", "--check", *pkg_strs)

    print("==> Ruff lint")
    run("ruff", "check", *pkg_strs)

    mypy_dirs = [d for d in pkg_dirs if files_in([d], "__init__.py")]
    print(f"==> mypy ({len(mypy_dirs)} packages)")
    if mypy_dirs:
        mypypath = ":".join(str(ROOT / d) for d in all_pkg_dirs)
        run("mypy", *[str(d) for d in mypy_dirs], env={**os.environ, "MYPYPATH": mypypath})


def lint_cpp(pkg_dirs: list[Path]) -> None:
    cpp = files_in(pkg_dirs, "*.cpp", "*.hpp", "*.h", "*.cc")
    print("==> clang-format (check)")
    run("clang-format", "--dry-run", "--Werror", *[str(f.relative_to(ROOT)) for f in cpp])


def main() -> None:
    parser = argparse.ArgumentParser(description="Run all linters")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--only", nargs="+", metavar="PKG")
    group.add_argument("--ignore", nargs="+", metavar="PKG")
    args = parser.parse_args()

    missing = [t for t in ("ruff", "mypy", "clang-format") if not shutil.which(t)]
    if missing:
        die(f"missing tools: {', '.join(missing)}. Run just setup")

    pkg_dirs, all_pkg_dirs = resolve_packages(args.only, args.ignore)

    if files_in(pkg_dirs, "*.py"):
        lint_python(pkg_dirs, all_pkg_dirs)

    if files_in(pkg_dirs, "*.cpp", "*.hpp", "*.h", "*.cc"):
        lint_cpp(pkg_dirs)

    print("\n==> All checks passed!")


if __name__ == "__main__":
    main()
