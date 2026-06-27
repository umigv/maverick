#!/usr/bin/env python3
import os
import shutil
import sys
from pathlib import Path

from common import ROOT, die, run

BASH_BODY = """\
# Managed by the maverick environment bootstrap.
_pixi_ros2_completion() {
    [ -n "$CONDA_PREFIX" ] || return
    [ "$_ROS2_COMPL" = "$CONDA_PREFIX" ] && return

    local f="$CONDA_PREFIX/share/ros2cli/environment/ros2-argcomplete.bash"
    [ -f "$f" ] || return

    . "$f"
    export _ROS2_COMPL="$CONDA_PREFIX"
}
PROMPT_COMMAND="${PROMPT_COMMAND:+$PROMPT_COMMAND;}_pixi_ros2_completion\""""

ZSH_BODY = """\
# Managed by the maverick environment bootstrap.
_pixi_ros2_completion() {
    [ -n "$CONDA_PREFIX" ] || return
    [ "$_ROS2_COMPL" = "$CONDA_PREFIX" ] && return

    local f="$CONDA_PREFIX/share/ros2cli/environment/ros2-argcomplete.zsh"
    [ -f "$f" ] || return

    source "$f"
    export _ROS2_COMPL="$CONDA_PREFIX"
}
if [[ ! " ${precmd_functions[*]} " =~ " _pixi_ros2_completion " ]]; then
    precmd_functions+=(_pixi_ros2_completion)
fi"""

MARK_START = "# >>> maverick environment >>>"
MARK_END = "# <<< maverick environment <<<"

GIT_HOOKS = {
    "pre-commit": "pre_commit.py",
    "post-checkout": "generate_pyrightconfig.py",
    "post-merge": "generate_pyrightconfig.py",
    "post-rewrite": "generate_pyrightconfig.py",
}


def log(msg: str) -> None:
    print(f"\033[1;34m===> {msg}\033[0m", flush=True)


def update_or_append_block(rc_path: Path, body: str) -> None:
    block = f"{MARK_START}\n{body}\n{MARK_END}"
    text = rc_path.read_text() if rc_path.exists() else ""
    if MARK_START in text:
        log(f"Updating configuration in ~/{rc_path.name}")
        before, _ = text.split(MARK_START, 1)
        _, after = text.split(MARK_END, 1)
        rc_path.write_text(before + block + after)
    else:
        log(f"Installing configuration in ~/{rc_path.name}")
        with rc_path.open("a") as f:
            f.write(f"\n{block}\n")


def configure_shell() -> None:
    if Path(os.environ.get("SHELL", "/bin/bash")).name == "zsh":
        update_or_append_block(Path.home() / ".zshrc", ZSH_BODY)
    else:
        update_or_append_block(Path.home() / ".bashrc", BASH_BODY)


def main() -> None:
    missing = [tool for tool in ("just", "direnv") if shutil.which(tool) is None]
    if missing:
        die(f"Missing system tools: {', '.join(missing)}. Run the host bootstrap first.")

    log("Initializing git submodules")
    if run("git", "submodule", "update", "--init", "--recursive", capture_output=True).strip():
        log("Submodules changed — clearing build, install, and log directories")
        for directory in ("build", "install", "log"):
            shutil.rmtree(ROOT / directory, ignore_errors=True)

    log("Allowing direnv in repo")
    run("direnv", "allow", str(ROOT))

    log("Configuring shell")
    configure_shell()

    log("Generating pyrightconfig.json")
    run(sys.executable, str(ROOT / "scripts" / "generate_pyrightconfig.py"))

    log("Installing git hooks")
    for hook, script in GIT_HOOKS.items():
        dest = ROOT / ".git" / "hooks" / hook
        dest.unlink(missing_ok=True)
        dest.symlink_to(ROOT / "scripts" / script)

    log("Setup complete. Open a new terminal for the changes to take effect.")


if __name__ == "__main__":
    main()
