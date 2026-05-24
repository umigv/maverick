#!/usr/bin/env python3
"""Shared utilities for scripts: ROOT path, subprocess helpers, and package discovery."""

import subprocess
import sys
from pathlib import Path
from typing import Literal, overload

ROOT = Path(__file__).resolve().parent.parent


def die(msg: str) -> None:
    """Print an error message to stderr and exit with code 1."""
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


@overload
def run(
    *cmd: str,
    env: dict | None = None,
    shell: bool = False,
    capture_output: Literal[True],
    cwd: Path = ROOT,
    stdin: str | None = None,
) -> str: ...
@overload
def run(
    *cmd: str,
    env: dict | None = None,
    shell: bool = False,
    capture_output: Literal[False] = False,
    cwd: Path = ROOT,
    stdin: str | None = None,
) -> None: ...
def run(
    *cmd: str,
    env: dict | None = None,
    shell: bool = False,
    capture_output: bool = False,
    cwd: Path = ROOT,
    stdin: str | None = None,
) -> str | None:
    """Run a command, exiting on failure. Defaults to ROOT if cwd is not specified.

    Returns stdout as a string if capture_output=True, otherwise None.
    """
    try:
        result = subprocess.run(
            cmd[0] if shell else cmd,
            shell=shell,
            check=True,
            cwd=cwd,
            env=env,
            capture_output=capture_output,
            text=capture_output or stdin is not None,
            input=stdin,
        )
        return result.stdout if capture_output else None
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)


def discover_packages() -> list[Path]:
    """Return paths to all ROS 2 packages under src/, relative to ROOT."""
    return sorted({p.parent.relative_to(ROOT) for p in (ROOT / "src").rglob("package.xml")})


def resolve_target(name: str, pkg_dirs: list[Path], extra_dirs: list[Path]) -> Path:
    """Resolve a package name to its path, dying if not found."""
    if Path(name) in extra_dirs:
        return Path(name)
    matches = [d for d in pkg_dirs if d.name == name]
    if not matches:
        die(f"'{name}' is not a valid target")
    return matches[0]


def resolve_packages(only: list[str] | None, ignore: list[str] | None) -> tuple[list[Path], list[Path]]:
    """Resolve the target package list from --only/--ignore filters.

    Returns (pkg_dirs, all_pkg_dirs) where pkg_dirs is the filtered set to operate on and all_pkg_dirs is the full
    unfiltered set (used for cross-package type checking).
    """
    print("==> Discovering ROS packages")
    all_pkg_dirs = discover_packages()
    if not all_pkg_dirs:
        die("No package.xml found under src/")

    extra_dirs = [Path("scripts")]
    pkg_dirs = list(all_pkg_dirs) + extra_dirs

    if only:
        pkg_dirs = [resolve_target(name, all_pkg_dirs, extra_dirs) for name in only]
    elif ignore:
        ignored = {resolve_target(name, all_pkg_dirs, extra_dirs) for name in ignore}
        pkg_dirs = [p for p in pkg_dirs if p not in ignored]

    if not pkg_dirs:
        die("No packages left after filtering")

    return pkg_dirs, all_pkg_dirs


def files_in(dirs: list[Path], *patterns: str) -> list[Path]:
    """Return all files matching any of the given glob patterns within the given directories."""
    return [f for d in dirs for pattern in patterns for f in (ROOT / d).rglob(pattern)]
