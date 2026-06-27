import json
from pathlib import Path
from typing import Literal, cast, get_args

import yaml
from ament_index_python.packages import get_package_share_directory

Mode = Literal["autonav", "self_drive", "nav_test"]
MODES: list[Mode] = list(get_args(Mode))


def format_mode_description(descriptions: dict[Mode, str]) -> str:
    if missing := set(MODES) - descriptions.keys():
        raise ValueError(f"format_mode_description: missing modes: {missing}")
    mode_pad = max(len(m) for m in MODES) + 2  # +2 for ": "
    lines = [f"{mode}:{' ' * (mode_pad - len(mode) - 1)}{descriptions[mode]}" for mode in MODES]
    return "\n        ".join(lines) + "\n        "


def bringup_share() -> str:
    return str(get_package_share_directory("bringup"))


def load_frames() -> dict:
    with Path(f"{bringup_share()}/config/frames.yaml").open() as f:
        return cast(dict, yaml.safe_load(f))


def load_gps_file(course: str) -> dict:
    with Path(f"{bringup_share()}/courses/{course}/gps.json").open() as f:
        return cast(dict, json.load(f))
