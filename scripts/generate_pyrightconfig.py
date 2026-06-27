#!/usr/bin/env python3
import json

from common import ROOT


def main():
    extra_paths = []
    for pkg_xml in sorted((ROOT / "src").rglob("package.xml")):
        pkg_dir = pkg_xml.parent
        if any(pkg_dir.rglob("__init__.py")):
            extra_paths.append(str(pkg_dir.relative_to(ROOT)))

    config = {
        "extraPaths": extra_paths,
        "pythonVersion": "3.10",
        "typeCheckingMode": "standard",
    }

    out = ROOT / "pyrightconfig.json"
    out.write_text(json.dumps(config, indent=4) + "\n")
    print(f"Written {out.name} with {len(extra_paths)} package paths")


if __name__ == "__main__":
    main()
