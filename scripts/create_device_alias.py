#!/usr/bin/env python3
import sys
from pathlib import Path

from common import run


def main() -> None:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <device> <alias> (e.g. {sys.argv[0]} /dev/ttyUSB0 imu)")
        sys.exit(1)

    dev = sys.argv[1]
    alias = sys.argv[2]
    rules_file = Path(f"/etc/udev/rules.d/99-{alias}.rules")

    if not Path(dev).exists():
        print(f"ERROR: Device {dev} not found.", file=sys.stderr)
        sys.exit(1)

    if rules_file.exists():
        print(f"Replacing existing rule at {rules_file}:")
        print(f"  {rules_file.read_text().strip()}")

    output = run("udevadm", "info", "--query=property", f"--name={dev}", capture_output=True)
    props = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)

    vendor = props.get("ID_VENDOR_ID", "")
    product = props.get("ID_MODEL_ID", "")
    serial = props.get("ID_SERIAL_SHORT", "")

    if not vendor or not product:
        print(f"ERROR: Could not read idVendor/idProduct from {dev}.", file=sys.stderr)
        sys.exit(1)

    print(f"Device:  {dev}")
    print(f"Alias:   /dev/{alias}")
    print(f"Vendor:  {vendor}")
    print(f"Product: {product}")
    print(f"Serial:  {serial or '(none found - using Vendor/Product only)'}")

    if serial and ":" not in serial:
        rule = f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", ATTRS{{serial}}=="{serial}", SYMLINK+="{alias}", MODE="0666"'
    else:
        rule = f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", SYMLINK+="{alias}", MODE="0666"'

    run("sudo", "tee", str(rules_file), stdin=rule + "\n", capture_output=True)


if __name__ == "__main__":
    main()
