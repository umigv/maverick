#!/usr/bin/env python3
import argparse
from pathlib import Path

from common import die, info, run, warning


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a udev alias for a device.")
    parser.add_argument("device", help="device path, e.g. /dev/ttyUSB0")
    parser.add_argument("alias", help="alias name, e.g. imu (creates /dev/imu)")
    args = parser.parse_args()

    rules_file = Path(f"/etc/udev/rules.d/99-{args.alias}.rules")

    if not Path(args.device).exists():
        die(f"Device {args.device} not found.")

    if rules_file.exists():
        warning(f"Replacing existing rule at {rules_file}:\n{rules_file.read_text().strip()}")

    output = run("udevadm", "info", "--query=property", f"--name={args.device}", capture_output=True)
    props = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)

    vendor = props.get("ID_VENDOR_ID", "")
    product = props.get("ID_MODEL_ID", "")
    serial = props.get("ID_SERIAL_SHORT", "")

    if not vendor or not product:
        die(f"Could not read idVendor/idProduct from {args.device}.")

    info(
        f"Detected device properties:\n"
        f"Device:  {args.device}\n"
        f"Alias:   /dev/{args.alias}\n"
        f"Vendor:  {vendor}\n"
        f"Product: {product}\n"
        f"Serial:  {serial or '(none)'}"
    )

    if not serial:
        warning("No serial number found - the rule will match any device with this vendor/product ID.")

    if serial and ":" not in serial:
        rule = f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", ATTRS{{serial}}=="{serial}", SYMLINK+="{args.alias}", MODE="0666"'
    else:
        rule = f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", SYMLINK+="{args.alias}", MODE="0666"'

    run("sudo", "tee", str(rules_file), stdin=rule + "\n", capture_output=True)


if __name__ == "__main__":
    main()
