from dataclasses import dataclass


@dataclass(frozen=True)
class SelfDriveNodeConfig:
    function_type: str
    hsv_json_key: str
