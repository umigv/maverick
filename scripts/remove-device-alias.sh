#!/usr/bin/env bash
set -euo pipefail

# Usage: ./remove-device-alias.sh <alias>
# Removes the permanent /dev/<alias> symlink for the given alias.
# Example: ./remove-device-alias.sh imu

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <alias> (e.g. $0 imu)"
    exit 1
fi

ALIAS="$1"
RULES_FILE="/etc/udev/rules.d/99-${ALIAS}.rules"

if [[ ! -f "$RULES_FILE" ]]; then
    echo "ERROR: No rule found for alias '$ALIAS' (expected $RULES_FILE)."
    exit 1
fi

echo "Removing rule at $RULES_FILE:"
echo "  $(cat "$RULES_FILE")"

sudo rm "$RULES_FILE"

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Udev rule removed. /dev/$ALIAS will no longer be created."
