#!/usr/bin/env python3
import odrive
import odrive.utils
from odrive.enums import AxisState


def calibrate_odrive(odrv) -> None:
    print(f"Calibrating ODrive {odrv.serial_number}...")
    print("There should be a beep and the indicator lights should flash green. Do not touch the robot!")
    odrive.utils.run_state(odrv.axis0, AxisState.FULL_CALIBRATION_SEQUENCE)
    print(f"ODrive {odrv.serial_number} calibrated!")


def main():
    print("Finding 2 ODrives...")
    odrv0, odrv1 = odrive.find_any(count=2)
    calibrate_odrive(odrv0)
    calibrate_odrive(odrv1)
    print("Make sure the indicator lights are blue")


if __name__ == "__main__":
    main()
