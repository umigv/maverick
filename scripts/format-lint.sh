#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "$ROOT"

command -v shfmt >/dev/null || {
    echo "shfmt not found. Run scripts/setup.sh"
    exit 1
}
command -v ruff >/dev/null || {
    echo "ruff not found. Run scripts/setup.sh"
    exit 1
}

ONLY_PKGS=()
IGNORE_PKGS=()

usage() {
    cat <<EOF
Usage: $(basename "$0") [--only pkg1 pkg2 ...] [--ignore pkg1 pkg2 ...]

Options:
  --only     Auto-fix only the specified packages
  --ignore   Auto-fix all discovered packages except the specified ones

Examples:
  $0
  $0 --only utils occupancy_grid_transform
  $0 --ignore perception cv_stack
EOF
    exit 1
}

# ---- parse args ----
while [[ $# -gt 0 ]]; do
    case "$1" in
    --only)
        shift
        while [[ $# -gt 0 && "$1" != --* ]]; do
            ONLY_PKGS+=("$1")
            shift
        done
        ;;
    --ignore)
        shift
        while [[ $# -gt 0 && "$1" != --* ]]; do
            IGNORE_PKGS+=("$1")
            shift
        done
        ;;
    -h | --help)
        usage
        ;;
    *)
        echo "Unknown argument: $1"
        usage
        ;;
    esac
done

if [[ "${#ONLY_PKGS[@]}" -gt 0 && "${#IGNORE_PKGS[@]}" -gt 0 ]]; then
    echo "ERROR: --only and --ignore are mutually exclusive"
    exit 1
fi

# ---- resolve targets (default: .) ----
TARGETS=(".")
if [[ "${#ONLY_PKGS[@]}" -gt 0 || "${#IGNORE_PKGS[@]}" -gt 0 ]]; then
    echo "==> Discovering ROS packages"
    mapfile -t ALL_PKG_DIRS < <(
        find src -type f -name package.xml -print0 |
            xargs -0 -n1 dirname |
            sort -u
    )

    if [[ "${#ALL_PKG_DIRS[@]}" -eq 0 ]]; then
        echo "ERROR: No <pkg>/package.xml found under project root"
        exit 1
    fi

    PKG_DIRS=("${ALL_PKG_DIRS[@]}")

    if [[ "${#ONLY_PKGS[@]}" -gt 0 ]]; then
        PKG_DIRS=()
        for pkg in "${ONLY_PKGS[@]}"; do
            if [[ -d "$pkg" && -f "$pkg/package.xml" ]]; then
                PKG_DIRS+=("$pkg")
            else
                echo "ERROR: '$pkg' is not a valid ROS package directory"
                exit 1
            fi
        done
    else
        FILTERED=()
        for dir in "${ALL_PKG_DIRS[@]}"; do
            skip=false
            for ignore in "${IGNORE_PKGS[@]}"; do
                if [[ "$dir" == "$ignore" ]]; then
                    skip=true
                    break
                fi
            done
            $skip || FILTERED+=("$dir")
        done
        PKG_DIRS=("${FILTERED[@]}")
    fi

    if [[ "${#PKG_DIRS[@]}" -eq 0 ]]; then
        echo "ERROR: No packages left to lint after filtering"
        exit 1
    fi

    TARGETS=("${PKG_DIRS[@]}")
fi

echo "==> shfmt"
shfmt -w scripts/

echo "==> Ruff lint (fix: imports + safe fixes)"
FIX_SUMMARY="$(ruff check --fix --exit-zero "${TARGETS[@]}" 2>/dev/null | grep -E 'fixed' || true)"

if [[ -n "$FIX_SUMMARY" ]]; then
    echo "$FIX_SUMMARY"
else
    echo "No lint fixes applied"
fi

echo "==> Ruff format"
ruff format "${TARGETS[@]}"
