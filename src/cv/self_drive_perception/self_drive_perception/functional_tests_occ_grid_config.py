from dataclasses import dataclass


@dataclass(frozen=True)
class SelfDrivePerceptionConfig:
    function_type: str
    hsv_json_key: str

    def __post_init__(self) -> None:
        modes: list[str] = ["right", "left", "pedlangechange", "curvedlanekeep"]

        if self.function_type not in modes:
            raise ValueError(
                f"SelfDrivePerceptionConfig: {self.function_type} is not a functional test (must be one of: {modes})."
            )
