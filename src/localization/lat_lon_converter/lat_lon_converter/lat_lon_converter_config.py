from dataclasses import dataclass


@dataclass(frozen=True)
class LatLonConverterConfig:
    """Config for LatLonConverter.

    Attributes:
        datum: ENU origin as [latitude (deg), longitude (deg), altitude (m)]. Required.
    """

    datum: list[float]

    def __post_init__(self) -> None:
        if len(self.datum) != 3:
            raise ValueError("LatLonConverterConfig: datum must be [latitude, longitude, altitude]")
