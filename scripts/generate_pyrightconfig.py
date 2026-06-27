#!/usr/bin/env python3
import json
import re

from common import ROOT, die

VENV_PATH = ".pixi/envs"
VENV_NAME = "default"
LOCKFILE = ROOT / "pixi.lock"


def detect_python_version() -> str:
    m = re.search(r"/python-(\d+\.\d+)\.\d+-", LOCKFILE.read_text())
    if m is None:
        die(f"Could not find python package in {LOCKFILE.name}")
    return m.group(1)


def main() -> None:
    extra_paths = []
    for pkg_xml in sorted((ROOT / "src").rglob("package.xml")):
        pkg_dir = pkg_xml.parent
        if any(pkg_dir.rglob("__init__.py")):
            extra_paths.append(str(pkg_dir.relative_to(ROOT)))

    config = {
        "venvPath": VENV_PATH,
        "venv": VENV_NAME,
        "extraPaths": extra_paths,
        "pythonVersion": detect_python_version(),
        "typeCheckingMode": "standard",
    }

    out = ROOT / "pyrightconfig.json"
    out.write_text(json.dumps(config, indent=4) + "\n")
    print(f"Written {out.name} with {len(extra_paths)} package paths")


if __name__ == "__main__":
    main()
