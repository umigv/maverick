#!/usr/bin/env python3
import os
import shutil
import sys
from pathlib import Path

from common import ROOT, die, info, run, warning

ROS2_COMPLETION_FILES = {
    "bash": (Path(".local/share/bash-completion/completions/ros2"), ()),
    "zsh": (Path(".zsh/completions/_ros2"), ()),
}

GIT_HOOKS = {
    "pre-commit": "pre_commit.py",
    "post-checkout": "generate_pyrightconfig.py",
    "post-merge": "generate_pyrightconfig.py",
    "post-rewrite": "generate_pyrightconfig.py",
}


def install_ros2_completion() -> None:
    shell = Path(os.environ.get("SHELL", "")).name
    if shell not in ROS2_COMPLETION_FILES:
        if not shell:
            reason = "Could not detect your login shell ($SHELL is not set)"
        else:
            reason = f"Login shell '{shell}' is not bash or zsh"
        warning(
            f"{reason} - skipping ros2 completion setup\n"
            "Register 'register-python-argcomplete ros2' output with your shell's completion system"
        )
        return
    if shutil.which("register-python-argcomplete") is None:
        warning(
            "register-python-argcomplete not found - skipping ros2 completion setup\n"
            "Run this script via 'just setup' so the pixi environment is on PATH"
        )
        return

    dest, args = ROS2_COMPLETION_FILES[shell]
    info(f"Installing ~/{dest}")
    path = Path.home() / dest
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(run("register-python-argcomplete", *args, "ros2", capture_output=True))


def main() -> None:
    missing = [tool for tool in ("just", "direnv") if shutil.which(tool) is None]
    if missing:
        die(f"Missing system tools: {', '.join(missing)}. Run the host bootstrap first.")

    info("Initializing git submodules")
    if run("git", "submodule", "update", "--init", "--recursive", capture_output=True).strip():
        info("Submodules changed - clearing build, install, and log directories")
        for directory in ("build", "install", "log"):
            shutil.rmtree(ROOT / directory, ignore_errors=True)

    info("Allowing direnv in repo")
    run("direnv", "allow", str(ROOT))

    info("Configuring ros2 completions")
    install_ros2_completion()

    info("Generating pyrightconfig.json")
    run(sys.executable, str(ROOT / "scripts" / "generate_pyrightconfig.py"))

    info("Installing git hooks")
    for hook, script in GIT_HOOKS.items():
        dest = ROOT / ".git" / "hooks" / hook
        dest.unlink(missing_ok=True)
        dest.symlink_to(ROOT / "scripts" / script)

    info("Setup complete. Open a new terminal for the changes to take effect")


if __name__ == "__main__":
    main()
