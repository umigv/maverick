from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EstopDriverConfig:
    """Config for EstopDriver.

    Attributes:
        serial_port: Path to the serial device.
        baud_rate: Serial baud rate for communication.
        poll_period_s: Period (s) at which the serial port is polled for new data.
        serial_timeout_s: Maximum interval without valid serial state (s).
    """

    serial_port: Path = Path("/dev/estop")
    baud_rate: int = 9600
    poll_period_s: float = 0.05
    serial_timeout_s: float = 1.0

    def __post_init__(self) -> None:
        if self.baud_rate <= 0:
            raise ValueError("EstopDriverConfig: baud_rate must be > 0")
        if self.poll_period_s <= 0:
            raise ValueError("EstopDriverConfig: poll_period_s must be > 0")
        if self.serial_timeout_s <= 0:
            raise ValueError("EstopDriverConfig: serial_timeout_s must be > 0")
