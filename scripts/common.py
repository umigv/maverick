#!/usr/bin/env python3
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Literal, NoReturn, overload

ROOT = Path(__file__).resolve().parent.parent


def die(msg: str) -> NoReturn:
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
        # A non-captured command already streamed its own errors to the terminal, so just relay its exit code;
        # logging here would only stack redundant messages up nested run() chains. For a captured command the
        # output was swallowed, so surface it (plus a summary, since some tools exit nonzero with no stderr).
        if capture_output:
            if e.stdout:
                print(e.stdout, file=sys.stderr, end="")
            if e.stderr:
                print(e.stderr, file=sys.stderr, end="")
            die(f"command failed (exit {e.returncode}): {shlex.join(cmd)}")
        sys.exit(e.returncode)


# Flat directories that are lint/format targets but not ROS packages (no package.xml).
EXTRA_DIRS = [Path("scripts")]


def discover_packages() -> list[Path]:
    """Return all lint/format targets relative to ROOT: ROS 2 packages (package.xml dirs under src/) plus EXTRA_DIRS."""
    ros = sorted({p.parent.relative_to(ROOT) for p in (ROOT / "src").rglob("package.xml")})
    return ros + EXTRA_DIRS


def resolve_target(name: str, pkg_dirs: list[Path]) -> Path:
    """Resolve a target name to its path, dying if not found."""
    matches = [d for d in pkg_dirs if d.name == name]
    if not matches:
        die(f"'{name}' is not a valid target")
    return matches[0]


def file_to_package(filepath: Path, pkg_dirs: list[Path]) -> str | None:
    """Return the name of the package a file belongs to, or None if it is outside all targets.

    The inverse of resolve_target. Deepest match wins so nested packages resolve to the innermost one.
    """
    for pkg_dir in sorted(pkg_dirs, key=lambda p: len(p.parts), reverse=True):
        if filepath == pkg_dir or pkg_dir in filepath.parents:
            return pkg_dir.name

    return None


def resolve_packages(only: list[str] | None, ignore: list[str] | None) -> tuple[list[Path], list[Path]]:
    """Resolve the target list from --only/--ignore filters.

    Returns (pkg_dirs, all_pkg_dirs) where pkg_dirs is the filtered set to operate on and all_pkg_dirs is the full
    unfiltered set of targets (ROS packages plus EXTRA_DIRS), used as the MYPYPATH bases for type checking.
    """
    print("==> Discovering packages")
    all_pkg_dirs = discover_packages()

    pkg_dirs = all_pkg_dirs
    if only:
        pkg_dirs = [resolve_target(name, all_pkg_dirs) for name in only]
    elif ignore:
        ignored = {resolve_target(name, all_pkg_dirs) for name in ignore}
        pkg_dirs = [p for p in pkg_dirs if p not in ignored]

    if not pkg_dirs:
        die("No packages left after filtering")

    return pkg_dirs, all_pkg_dirs


def files_in(dirs: list[Path], *patterns: str) -> list[Path]:
    """Return all files matching any of the given glob patterns within the given directories."""
    return [f for d in dirs for pattern in patterns for f in (ROOT / d).rglob(pattern)]
