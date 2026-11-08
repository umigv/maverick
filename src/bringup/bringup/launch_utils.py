import json
from pathlib import Path
from typing import Literal, cast, get_args

import yaml
from ament_index_python.packages import get_package_share_directory

Mode = Literal["autonav", "self_drive", "nav_test"]
MODES: list[Mode] = list(get_args(Mode))


def bringup_share() -> str:
    return str(get_package_share_directory("bringup"))


def load_frames() -> dict:
    with Path(f"{bringup_share()}/config/frames.yaml").open() as f:
        return cast(dict, yaml.safe_load(f))


def gps_file_path(course: str) -> str:
    return f"{bringup_share()}/courses/{course}/gps.json"


def load_gps_file(course: str) -> dict:
    with Path(gps_file_path(course)).open() as f:
        return cast(dict, json.load(f))
