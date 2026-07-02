#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

log() { printf '\033[1;34m===> %s\033[0m\n' "$*"; }
trap 'printf "\033[1;31mERROR at line %s: %s (exit %s)\033[0m\n" "$LINENO" "$BASH_COMMAND" "$?" >&2' ERR

# Single-quoted so $CONDA_PREFIX/$f/etc. stay literal for the user's shell to evaluate.
# shellcheck disable=SC2016
BASH_BODY='# Managed by the maverick environment bootstrap.
_pixi_ros2_completion() {
    [ -n "$CONDA_PREFIX" ] || return
    [ "$_ROS2_COMPL" = "$CONDA_PREFIX" ] && return

    local f="$CONDA_PREFIX/share/ros2cli/environment/ros2-argcomplete.bash"
    [ -f "$f" ] || return

    . "$f"
    export _ROS2_COMPL="$CONDA_PREFIX"
}
PROMPT_COMMAND="${PROMPT_COMMAND:+$PROMPT_COMMAND;}_pixi_ros2_completion"'

# shellcheck disable=SC2016
ZSH_BODY='# Managed by the maverick environment bootstrap.
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
fi'


update_or_append_block() {
    local rc_path="$1" body="$2" rc_name="${1##*/}"
    local mark_start='# >>> maverick environment >>>'
    local mark_end='# <<< maverick environment <<<'
    local block="$mark_start
$body
$mark_end"

    if grep -qF "$mark_start" "$rc_path"; then
        log "Updating configuration in ~/$rc_name"
        local tmp_block tmp_out
        tmp_block=$(mktemp)
        tmp_out=$(mktemp)
        printf '%s\n' "$block" > "$tmp_block"
        awk -v s="$mark_start" -v e="$mark_end" -v bf="$tmp_block" '
            $0 == s { while ((getline line < bf) > 0) print line; inside = 1; next }
            inside && $0 == e { inside = 0; next }
            !inside { print }
        ' "$rc_path" > "$tmp_out"
        mv "$tmp_out" "$rc_path"
        rm -f "$tmp_block"
    else
        log "Installing configuration in ~/$rc_name"
        printf '\n%s\n' "$block" >> "$rc_path"
    fi
}

configure_shell() {
    local rc body
    case "$(basename "${SHELL:-/bin/bash}")" in
        zsh) rc="$HOME/.zshrc";  body="$ZSH_BODY" ;;
        *)   rc="$HOME/.bashrc"; body="$BASH_BODY" ;;
    esac
    touch "$rc"
    update_or_append_block "$rc" "$body"
}

# SIGHUP the terminal emulator so its window closes, forcing a fresh login shell so the rc changes we just wrote take
# effect. We climb the process tree by ancestor *name* rather than a fixed number of hops, since the launch chain
# between this script and the terminal can gain or lose layers.
close_terminal() {
    local pid=$PPID name
    while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
        name=$(ps -o comm= -p "$pid" 2>/dev/null) || break
        case "$name" in
            *gnome-terminal*|*konsole*|*xterm*|*alacritty*|*kitty*|*tilix*|\
            *terminator*|*wezterm*|*foot*|*ptyxis*|*xfce4-terminal|login**)
                kill -HUP "$pid"
                return 0 ;;
        esac
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    done

    echo "close_terminal: unrecognized terminal emulator in process ancestry; leaving open" >&2
    return 1
}

main() {
    missing=()
    for tool in just direnv; do
        command -v "$tool" &>/dev/null || missing+=("$tool")
    done
    if [ ${#missing[@]} -gt 0 ]; then
        echo "Missing system tools: ${missing[*]}. Run the host bootstrap first." >&2
        exit 1
    fi

    log "Initializing git submodules"
    submodule_out=$(git -C "$ROOT" submodule update --init --recursive)
    if [ -n "$(printf '%s' "$submodule_out" | tr -d '[:space:]')" ]; then
        log "Submodules changed — clearing build, install, and log directories"
        rm -rf "$ROOT/build" "$ROOT/install" "$ROOT/log"
    fi

    if ! command -v pixi &>/dev/null; then
        log "Installing pixi"
        curl -fsSL https://pixi.sh/install.sh | bash
        export PATH="${PIXI_BIN_DIR:-$HOME/.pixi/bin}:$PATH"
    fi

    log "Installing dependencies via pixi"
    pixi install

    log "Allowing direnv in repo"
    direnv allow "$ROOT"

    log "Configuring shell"
    configure_shell

    log "Generating pyrightconfig.json"
    just generate-pyrightconfig

    log "Installing git hooks"
    install_hook() {
        rm -f "$ROOT/.git/hooks/$1"
        ln -s "$2" "$ROOT/.git/hooks/$1"
    }
    install_hook pre-commit    "$ROOT/scripts/pre_commit.py"
    install_hook post-checkout "$ROOT/scripts/generate_pyrightconfig.py"
    install_hook post-merge    "$ROOT/scripts/generate_pyrightconfig.py"
    install_hook post-rewrite  "$ROOT/scripts/generate_pyrightconfig.py"

    log "Setup complete. Press enter to close this terminal (required)."
    read -r
    # Setup already succeeded; a failed auto-close shouldn't fail the script.
    close_terminal || true
}

main "$@"
