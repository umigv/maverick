from dataclasses import dataclass


@dataclass(frozen=True)
class UbloxDriverConfig:
    """Configuration for the ublox driver node.

    Attributes:
        serial_port: The serial port device path for the GPS receiver.
        poll_period_s: The period in seconds between GPS polls.
        ublox_frame_id: The TF frame ID to use in published GPS messages.
    """

    serial_port: str = "/dev/ublox"
    poll_period_s: float = 0.1
    ublox_frame_id: str = "gps_link"

    def __post_init__(self) -> None:
        if self.poll_period_s <= 0:
            raise ValueError("UbloxDriverConfig: poll_period_s must be > 0")
