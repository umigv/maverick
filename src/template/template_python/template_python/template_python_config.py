from dataclasses import dataclass


@dataclass(frozen=True)
class TemplatePythonConfig:
    """Config for TemplatePython. See utils.config for supported field types and the YAML parameter mapping.

    Attributes:
        (document each field here)
    """

    def __post_init__(self) -> None:
        # Validate fields here, e.g. raise ValueError on out-of-range values.
        pass
