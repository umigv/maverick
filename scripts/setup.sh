#!/usr/bin/env bash
set -euo pipefail

BOLD='\033[1m'
CYAN='\033[0;36m'
YELLOW='\033[0;33m'
RESET='\033[0m'

log()  { echo -e "\n${BOLD}${CYAN}==> $*${RESET}"; }
note() { echo -e "    ${YELLOW}NOTE: $*${RESET}"; }
err()  { echo -e "\n${BOLD}ERROR: $*${RESET}" >&2; }

export ROS_VERSION=2
export ROS_DISTRO="${ROS_DISTRO:=humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  err "ROS setup not found at $ROS_SETUP"
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

log "Repo root: $REPO_ROOT"

if [[ ! -d "$REPO_ROOT/src" ]]; then
  err "Expected workspace src/ at: $REPO_ROOT/src"
  exit 1
fi

log "Initializing git submodules"
submodule_output=$(git -C "$REPO_ROOT" submodule update --init --recursive)
if [[ -n "$submodule_output" ]]; then
  log "Submodules changed — clearing build, install, and log directories"
  rm -rf "$REPO_ROOT/build" "$REPO_ROOT/install" "$REPO_ROOT/log"
fi

log "Installing apt tooling deps"
sudo apt update
sudo xargs apt-get install -y < "$REPO_ROOT/tooling.apt"

if ! command -v just >/dev/null 2>&1; then
    log "Installing just"
    curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/local/bin
fi

log "Configuring direnv shell hook"
if ! grep -q 'direnv hook bash' ~/.bashrc; then
  echo 'export DIRENV_LOG_FORMAT=' >> ~/.bashrc
  echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
fi

log "Installing Python tooling deps"
python3 -m pip install -e "${REPO_ROOT}[tooling]"

log "Installing ROS deps via rosdep"
rosdep update
rosdep install --from-paths "$REPO_ROOT" --ignore-src -r -y

log "Allowing direnv in repo"
direnv allow "$REPO_ROOT"

log "Generating pyrightconfig.json"
python3 "$REPO_ROOT/scripts/generate_pyrightconfig.py"

log "Setup complete"
note "Run source ~/.bashrc for direnv hook to take effect"
