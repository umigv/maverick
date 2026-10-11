from dataclasses import dataclass

import pyzed.sl as sl


@dataclass(frozen=True)
class ZedPublisherConfig:
    resolution: sl.RESOLUTION = sl.RESOLUTION.HD720
    fps: int = 30
    depth_mode: sl.DEPTH_MODE = sl.DEPTH_MODE.NEURAL

    brightness: int = 4  # 0-8
    contrast: int = 4  # 0-8
    hue: int = 0  # 0-11
    saturation: int = 4  # 0-8
    sharpness: int = 4  # 0-8
    gamma: int = 5  # 1-9

    gain: int = 50  # 0-100
    exposure: int = 50  # 0-100

    white_balance_temperature: int = 4600  # 2800-6500

    auto_exposure: bool = True
    auto_white_balance: bool = True

    def __post_init__(self) -> None:
        if self.brightness < 0 or self.brightness > 8:
            raise ValueError(f"[ZedPublisherConfig] Brightness must be between 0-8 (given: {self.brightness})")

        if self.contrast < 0 or self.contrast > 8:
            raise ValueError(f"[ZedPublisherConfig] Contrast must be between 0-8 (given: {self.contrast})")

        if self.hue < 0 or self.hue > 11:
            raise ValueError(f"[ZedPublisherConfig] Hue must be between 0-11 (given: {self.hue})")

        if self.saturation < 0 or self.saturation > 8:
            raise ValueError(f"[ZedPublisherConfig] Saturation must be between 0-8 (given: {self.saturation})")

        if self.sharpness < 0 or self.sharpness > 8:
            raise ValueError(f"[ZedPublisherConfig] Sharpness must be between 0-8 (given: {self.sharpness})")

        if self.gamma < 1 or self.gamma > 9:
            raise ValueError(f"[ZedPublisherConfig] Gamma must be between 1-9 (given: {self.gamma})")

        if self.gain < 0 or self.gain > 100:
            raise ValueError(f"[ZedPublisherConfig] Gain must be between 0-100 (given: {self.gain})")

        if self.exposure < 0 or self.exposure > 100:
            raise ValueError(f"[ZedPublisherConfig] Exposure must be between 0-100 (given: {self.exposure})")

        if self.white_balance_temperature < 2800 or self.white_balance_temperature > 6500:
            raise ValueError(
                "[ZedPublisherConfig] White balance temperature must be "
                f"between 2800-6500 (given: {self.white_balance_temperature})"
            )
