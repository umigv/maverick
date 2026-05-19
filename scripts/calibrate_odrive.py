#!/usr/bin/env python3
import odrive
import odrive.utils
from odrive.enums import AxisState


def calibrate_odrive(serial_number: str) -> None:
    print(f"Finding ODrive with serial number {serial_number}...")
    odrv = odrive.find_any(serial_number=serial_number)

    print("There should be a beep and the indicator lights should flash green. Do not touch the robot!")
    odrive.utils.run_state(odrv.axis0, AxisState.FULL_CALIBRATION_SEQUENCE)  # type: ignore[reportUnusedCoroutine]

    print(f"ODrive {serial_number} calibrated!")


def main():
    calibrate_odrive("395534753331")  # Left
    calibrate_odrive("384934743539")  # Right
    print("Make sure the indicator lights are blue")


if __name__ == "__main__":
    main()
