from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class LedDriverConfig:
    """Config for LedDriver.

    Attributes:
        estop_file_path: Path to the file containing the e-stop state ("1" = stopped).
        serial_port: Path to the serial device.
        baud_rate: Serial baud rate for communication.
        ready_timeout_s: Seconds to wait for the READY message before giving up.
        cmd_vel_timeout_s: Seconds after the last teleop/nav command before that source is considered inactive.
        update_period_s: Period (s) at which the LED state is evaluated and sent.
    """

    estop_file_path: Path
    serial_port: Path = Path("/dev/led")
    baud_rate: int = 9600
    ready_timeout_s: float = 5.0
    cmd_vel_timeout_s: float = 1.0
    update_period_s: float = 0.1

    def __post_init__(self) -> None:
        if self.baud_rate <= 0:
            raise ValueError("LedDriverConfig: baud_rate must be > 0")
        if self.ready_timeout_s <= 0:
            raise ValueError("LedDriverConfig: ready_timeout_s must be > 0")
        if self.cmd_vel_timeout_s <= 0:
            raise ValueError("LedDriverConfig: cmd_vel_timeout_s must be > 0")
        if self.update_period_s <= 0:
            raise ValueError("LedDriverConfig: update_period_s must be > 0")
