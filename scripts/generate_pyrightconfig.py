#!/usr/bin/env python3
"""Generate pyrightconfig.json from packages found in src/."""

import json
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).parent.parent


def main():
    result = subprocess.run(
        ["find", "src", "-type", "f", "-name", "package.xml"],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    extra_paths = []
    for line in sorted(result.stdout.strip().splitlines()):
        pkg_dir = Path(line).parent
        if any((REPO_ROOT / pkg_dir).rglob("__init__.py")):
            extra_paths.append(str(pkg_dir))

    config = {
        "extraPaths": extra_paths,
        "pythonVersion": "3.10",
    }

    out = REPO_ROOT / "pyrightconfig.json"
    out.write_text(json.dumps(config, indent=4) + "\n")
    print(f"Written {out.name} with {len(extra_paths)} package paths")


if __name__ == "__main__":
    main()
