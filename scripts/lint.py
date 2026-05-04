#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def die(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def run(*cmd: str, env: dict | None = None) -> None:
    subprocess.run(cmd, check=True, cwd=ROOT, env=env)


def discover_packages() -> list[Path]:
    return sorted({p.parent.relative_to(ROOT) for p in (ROOT / "src").rglob("package.xml")})


def get_submodule_dirs() -> list[Path]:
    result = subprocess.run(
        ["git", "submodule", "foreach", "--quiet", "echo $displaypath"],
        capture_output=True,
        text=True,
        cwd=ROOT,
    )
    return [Path(line) for line in result.stdout.splitlines() if line]


def resolve_target(name: str, pkg_dirs: list[Path], extra_dirs: list[Path]) -> Path:
    if Path(name) in extra_dirs:
        return Path(name)
    matches = [d for d in pkg_dirs if d.name == name]
    if not matches:
        die(f"'{name}' is not a valid target")
    return matches[0]


def files_in(dirs: list[Path], *patterns: str) -> list[Path]:
    return [f for d in dirs for pattern in patterns for f in (ROOT / d).rglob(pattern)]


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
        die(f"missing tools: {', '.join(missing)}. Run scripts/setup.py")

    print("==> Discovering ROS packages")
    all_pkg_dirs = discover_packages()
    if not all_pkg_dirs:
        die("No package.xml found under src/")

    subs = get_submodule_dirs()
    all_pkg_dirs = [p for p in all_pkg_dirs if not any(p == s or s in p.parents for s in subs)]
    if not all_pkg_dirs:
        die("No packages found after filtering submodules")

    extra_dirs = [Path("scripts")]

    pkg_dirs = list(all_pkg_dirs) + extra_dirs
    if args.only:
        pkg_dirs = [resolve_target(name, all_pkg_dirs, extra_dirs) for name in args.only]
    elif args.ignore:
        ignored = {resolve_target(name, all_pkg_dirs, extra_dirs) for name in args.ignore}
        pkg_dirs = [p for p in pkg_dirs if p not in ignored]

    if not pkg_dirs:
        die("No packages left to lint after filtering")

    if files_in(pkg_dirs, "*.py"):
        lint_python(pkg_dirs, all_pkg_dirs)

    if files_in(pkg_dirs, "*.cpp", "*.hpp", "*.h", "*.cc"):
        lint_cpp(pkg_dirs)

    print("\n==> All checks passed!")


if __name__ == "__main__":
    main()
