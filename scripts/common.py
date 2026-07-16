#!/usr/bin/env python3
import shlex
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, NoReturn, overload

ROOT = Path(__file__).resolve().parent.parent


def die(msg: str) -> NoReturn:
    """Print an error message to stderr in red and exit with code 1."""
    first, *rest = msg.splitlines()
    print(f"\033[1;31mERROR: {first}\033[0m", file=sys.stderr)
    for line in rest:
        print(f"\033[31m       {line}\033[0m", file=sys.stderr)
    sys.exit(1)


def info(msg: str) -> None:
    """Print a progress message in cyan."""
    first, *rest = msg.splitlines()
    print(f"\033[1;36m===> {first}\033[0m", flush=True)
    for line in rest:
        print(f"\033[36m     {line}\033[0m", flush=True)


def warning(msg: str) -> None:
    """Print a warning message to stderr in yellow."""
    first, *rest = msg.splitlines()
    print(f"\033[1;33mWARNING: {first}\033[0m", file=sys.stderr, flush=True)
    for line in rest:
        print(f"\033[33m         {line}\033[0m", file=sys.stderr, flush=True)


@overload
def run(
    *cmd: str,
    env: dict | None = None,
    capture_output: Literal[True],
    cwd: Path = ROOT,
    stdin: str | None = None,
) -> str: ...
@overload
def run(
    *cmd: str,
    env: dict | None = None,
    capture_output: Literal[False] = False,
    cwd: Path = ROOT,
    stdin: str | None = None,
) -> None: ...
def run(
    *cmd: str,
    env: dict | None = None,
    capture_output: bool = False,
    cwd: Path = ROOT,
    stdin: str | None = None,
) -> str | None:
    """Run a command, exiting on failure. Defaults to ROOT if cwd is not specified.

    Returns stdout as a string if capture_output=True, otherwise None.
    """
    try:
        result = subprocess.run(
            cmd,
            check=True,
            cwd=cwd,
            env=env,
            capture_output=capture_output,
            text=capture_output or stdin is not None,
            input=stdin,
        )
        return result.stdout if capture_output else None
    except subprocess.CalledProcessError as e:
        # A non-captured command already streamed its own errors to the terminal, so just relay its exit code. logging
        # here would only stack redundant messages up nested run() chains. For a captured command the output was
        # swallowed, so surface it (plus a summary, since some tools exit nonzero with no stderr).
        if capture_output:
            if e.stdout:
                print(e.stdout, file=sys.stderr, end="")
            if e.stderr:
                print(e.stderr, file=sys.stderr, end="")
            die(f"command failed (exit {e.returncode}): {shlex.join(cmd)}")
        sys.exit(e.returncode)


@dataclass(frozen=True)
class Target:
    path: Path  # relative to ROOT
    name: str = ""
    recursive: bool = True

    def __post_init__(self) -> None:
        if not self.name:
            object.__setattr__(self, "name", self.path.name)


# Flat directories that are lint/format targets but not ROS packages (no package.xml).
# TODO: add more targets such as waypoints/README.md
EXTRA_TARGETS = [Target(Path("scripts")), Target(Path(), "root", False)]


def discover_targets() -> list[Target]:
    """Return all lint/format targets relative to ROOT: ROS 2 packages (package.xml dirs under src/) plus EXTRA_TARGETS.

    Dies if two targets share a basename, since --only/--ignore and pre-commit address targets by name.
    """
    ros_pkgs = sorted({p.parent.relative_to(ROOT) for p in (ROOT / "src").rglob("package.xml")})
    targets = [Target(path) for path in ros_pkgs] + EXTRA_TARGETS

    counts = Counter(t.name for t in targets)
    if dupes := [name for name, n in counts.items() if n > 1]:
        die(f"Duplicate target names: {', '.join(sorted(dupes))}")

    return targets


def target_from_name(name: str, targets: list[Target]) -> Target:
    """Return the target directory with the given name, dying if not found."""
    matches = [t for t in targets if t.name == name]
    if not matches:
        die(f"'{name}' is not a valid target")
    return matches[0]


def target_from_file(file: Path, targets: list[Target]) -> Target | None:
    """Return the target directory a file belongs to, or None if it is outside all targets.

    Deepest match wins so nested packages resolve to the innermost one.
    """
    for target in sorted(targets, key=lambda p: len(p.path.parts), reverse=True):
        if file == target.path or target.path in (file.parents if target.recursive else [file.parent]):
            return target

    return None


def filter_targets(targets: list[Target], *, only: list[str] | None, ignore: list[str] | None) -> list[Target]:
    """Apply --only/--ignore name filters to a target list, dying if nothing is left."""
    if only and ignore:
        die("only and ignore are mutually exclusive")

    filtered = targets
    if only:
        # Dedupe repeated names: passing the same directory to mypy twice fails with "Duplicate module".
        filtered = list(dict.fromkeys(target_from_name(name, targets) for name in only))
    elif ignore:
        ignored = {target_from_name(name, targets) for name in ignore}
        filtered = [t for t in targets if t not in ignored]

    if not filtered:
        die("No targets left after filtering")

    return filtered


def files_in(targets: list[Target], *patterns: str) -> list[Path]:
    """Return all files matching any of the given glob patterns within the given targets."""
    return [
        f
        for t in targets
        for pattern in patterns
        for f in (ROOT / t.path).glob(("**/" if t.recursive else "") + pattern)
    ]
