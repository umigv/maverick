from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class GpsOriginCalculatorConfig:
    """Config for GpsOriginCalculator.

    Attributes:
        min_samples_required: Minimum number of valid GPS samples before setting datum.
        min_sample_duration_s: Minimum time (s) to collect samples.
        max_sample_duration_s: Maximum time (s) before setting datum regardless of sample count.
        max_horizontal_stdev_m: Maximum horizontal accuracy (m) for a sample to be accepted.
        output_file: Path to gps.json to write the computed datum into. Empty string skips writing.
    """

    min_samples_required: int = 3000
    min_sample_duration_s: float = 60.0
    max_sample_duration_s: float = 90.0
    max_horizontal_stdev_m: float = 1.0
    output_file: Path = Path("")

    def __post_init__(self) -> None:
        if self.min_samples_required <= 0:
            raise ValueError("GpsOriginCalculatorConfig: min_samples_required must be > 0")
        if self.min_sample_duration_s <= 0:
            raise ValueError("GpsOriginCalculatorConfig: min_sample_duration_s must be > 0")
        if self.max_sample_duration_s <= 0:
            raise ValueError("GpsOriginCalculatorConfig: max_sample_duration_s must be > 0")
        if self.min_sample_duration_s > self.max_sample_duration_s:
            raise ValueError("GpsOriginCalculatorConfig: max_sample_duration_s must be >= min_sample_duration_s")
        if self.max_horizontal_stdev_m <= 0:
            raise ValueError("GpsOriginCalculatorConfig: max_horizontal_stdev_m must be > 0")
