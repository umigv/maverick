#!/usr/bin/env python3
import odrive


def clear_odrive_errors(odrv) -> None:
    print(f"Clearing errors from ODrive {odrv.serial_number}...")
    odrv.clear_errors()
    print(f"Errors cleared from ODrive {odrv.serial_number}!")


def main():
    print("Finding 2 ODrives...")
    odrv0, odrv1 = odrive.find_any(count=2)
    clear_odrive_errors(odrv0)
    clear_odrive_errors(odrv1)
    print("Make sure the indicator lights are not flashing red")


if __name__ == "__main__":
    main()
