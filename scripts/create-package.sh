#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <package_name>"
  exit 1
fi

PKG_NAME="$1"

if [[ ! "$PKG_NAME" =~ ^[a-z][a-z0-9_]*$ ]]; then
  echo "ERROR: Invalid package name '$PKG_NAME'"
  echo "Package names must start with a lowercase letter and contain only lowercase letters, numbers, and underscores."
  exit 1
fi

echo "==> Creating ROS 2 Python package: $PKG_NAME"

ros2 pkg create \
  --build-type ament_python \
  --license Apache-2.0 \
  "$PKG_NAME"

rm -f "$PKG_NAME/LICENSE"
rm -rf "$PKG_NAME/test"

echo "==> Package '$PKG_NAME' created successfully"
