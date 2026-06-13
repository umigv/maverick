import json
from typing import Literal, get_args

import yaml
from ament_index_python.packages import get_package_share_directory

Mode = Literal["autonav", "self_drive", "nav_test"]
MODES: list[Mode] = list(get_args(Mode))

# File the e-stop state is shared through: estop_driver writes it, led_driver and odrive_driver read it ("1" = stopped)
ESTOP_FILE_PATH = "/tmp/estop_value.txt"


def bringup_share() -> str:
    return get_package_share_directory("bringup")


def load_frames() -> dict:
    with open(f"{bringup_share()}/config/frames.yaml") as f:
        return yaml.safe_load(f)


def gps_file_path(course: str) -> str:
    return f"{bringup_share()}/courses/{course}/gps.json"


def load_gps_file(course: str) -> dict:
    with open(gps_file_path(course)) as f:
        return json.load(f)
