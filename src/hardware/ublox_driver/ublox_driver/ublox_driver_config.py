from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class UbloxDriverConfig:
    """Configuration for the ublox driver node.

    Attributes:
        serial_port: The serial port device path for the GPS receiver.
        baud_rate: Serial baud rate for communication.
        poll_period_s: The period in seconds between GPS polls.
        ublox_frame_id: The TF frame ID to use in published GPS messages.
    """

    serial_port: Path = Path("/dev/ublox")
    baud_rate: int = 460800
    poll_period_s: float = 0.1
    ublox_frame_id: str = "gps_link"

    def __post_init__(self) -> None:
        if self.baud_rate <= 0:
            raise ValueError("UbloxDriverConfig: baud_rate must be > 0")
        if self.poll_period_s <= 0:
            raise ValueError("UbloxDriverConfig: poll_period_s must be > 0")
