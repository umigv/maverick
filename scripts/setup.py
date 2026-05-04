#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ROS_DISTRO = os.environ.get("ROS_DISTRO", "humble")


def log(msg: str) -> None:
    print(f"\n\033[1m\033[0;36m==> {msg}\033[0m")


def note(msg: str) -> None:
    print(f"    \033[0;33mNOTE: {msg}\033[0m")


def run(*cmd: str, **kwargs) -> None:
    subprocess.run(cmd, check=True, **kwargs)


def main() -> None:
    if not Path(f"/opt/ros/{ROS_DISTRO}").exists():
        print(f"ROS not found at /opt/ros/{ROS_DISTRO}", file=sys.stderr)
        sys.exit(1)

    if not (ROOT / "src").is_dir():
        print(f"Expected workspace src/ at: {ROOT}/src", file=sys.stderr)
        sys.exit(1)

    log(f"Repo root: {ROOT}")

    log("Initializing git submodules")
    result = subprocess.run(
        ["git", "-C", str(ROOT), "submodule", "update", "--init", "--recursive"],
        capture_output=True,
        text=True,
    )
    if result.stdout.strip():
        log("Submodules changed — clearing build, install, and log directories")
        for d in ("build", "install", "log"):
            shutil.rmtree(ROOT / d, ignore_errors=True)

    log("Installing apt tooling deps")
    run("sudo", "apt", "update")
    apt_packages = (ROOT / "tooling.apt").read_text().split()
    run("sudo", "apt-get", "install", "-y", *apt_packages)

    if not shutil.which("just"):
        log("Installing just")
        subprocess.run(
            "curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/local/bin",
            shell=True,
            check=True,
        )

    bashrc = Path.home() / ".bashrc"
    content = bashrc.read_text() if bashrc.exists() else ""

    if "just --completions bash" not in content:
        log("Configuring just autocomplete")
        with bashrc.open("a") as f:
            f.write("source <(just --completions bash)\n")

    if "direnv hook bash" not in content:
        log("Configuring direnv shell hook")
        with bashrc.open("a") as f:
            f.write('export DIRENV_LOG_FORMAT=\neval "$(direnv hook bash)"\n')

    log("Installing Python tooling deps")
    run(sys.executable, "-m", "pip", "install", "-e", f"{ROOT}[tooling]")

    log("Installing ROS deps via rosdep")
    env = {**os.environ, "ROS_DISTRO": ROS_DISTRO, "ROS_VERSION": "2"}
    run("rosdep", "update", env=env)
    run("rosdep", "install", "--from-paths", str(ROOT), "--ignore-src", "-r", "-y", env=env)

    log("Allowing direnv in repo")
    run("direnv", "allow", str(ROOT))

    log("Generating pyrightconfig.json")
    run(sys.executable, str(ROOT / "scripts" / "generate_pyrightconfig.py"))

    log("Configuring git hooks")
    run("git", "config", "core.hooksPath", "hooks")

    log("Setup complete")
    note("Run source ~/.bashrc for direnv hook to take effect")


if __name__ == "__main__":
    main()
