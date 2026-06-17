#!/usr/bin/env python3
import csv
from pathlib import Path

from common import ROOT, die

DMS_DIR = ROOT / "waypoints" / "dms"
DECIMAL_DIR = ROOT / "waypoints" / "decimal"

DmsWaypoint = tuple[str, float, float]  # (name, latitude, longitude) with packed-DMS coordinates
DecimalWaypoint = tuple[str, float, float]  # (name, latitude, longitude) in decimal degrees


def packed_dms_to_decimal(value: float) -> float:
    """Convert a packed DMS float (DD.MMSSssss) to decimal degrees."""
    sign = -1 if value < 0 else 1
    value = abs(value)

    degree = int(value)
    remainder = round((value - degree) * 100, 10)
    minute = int(remainder)
    second = round((remainder - minute) * 100, 10)

    return sign * (degree + minute / 60 + second / 3600)


def extract(input_path: Path) -> list[DmsWaypoint]:
    waypoints: list[DmsWaypoint] = []
    with input_path.open(newline="") as f:
        reader = csv.reader(f)

        header = next(reader, None)
        if header is None:
            die(f"{input_path} is empty")

        if [c.strip().lower() for c in header] != ["name", "latitude", "longitude"]:
            die(f"{input_path}:1: expected header 'name,latitude,longitude', got {header!r}")

        for i, row in enumerate(reader, start=2):
            if not row:
                continue

            if len(row) != 3:
                die(f"{input_path}:{i}: expected 3 columns (name,latitude,longitude), got {row!r}")

            name, latitude_str, longitude_str = (cell.strip() for cell in row)
            for label, raw in (("latitude", latitude_str), ("longitude", longitude_str)):
                try:
                    float(raw)
                except ValueError:
                    die(f"{input_path}:{i}: non-numeric {label} {raw!r}")

            waypoints.append((name, float(latitude_str), float(longitude_str)))

    return waypoints


def convert(waypoints: list[DmsWaypoint]) -> list[DecimalWaypoint]:
    return [
        (name, packed_dms_to_decimal(latitude), packed_dms_to_decimal(longitude))
        for name, latitude, longitude in waypoints
    ]


def write(waypoints: list[DecimalWaypoint], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["name", "latitude", "longitude"])
        for name, latitude, longitude in waypoints:
            # We pick 7dp for latitude / longitude here because
            # 1. It's around 1cm precision which is finer than what we can measure
            # 2. Doesn't match the 9dp precision of the data IGVC provides to prevent confusion
            # Keep in sync with gps_origin_calculator and course creation tool
            writer.writerow([name, f"{latitude:.7f}", f"{longitude:.7f}"])


def convert_file(input_path: Path) -> None:
    output_path = DECIMAL_DIR / input_path.name

    dms_waypoints = extract(input_path)
    waypoints = convert(dms_waypoints)
    write(waypoints, output_path)
    print(f"Converted {len(waypoints)} waypoint(s): {input_path.relative_to(ROOT)} -> {output_path.relative_to(ROOT)}")


def main() -> None:
    paths = sorted(DMS_DIR.glob("*.csv"))
    if not paths:
        die(f"no .csv files found in {DMS_DIR.relative_to(ROOT)}/")

    for path in paths:
        convert_file(path)


if __name__ == "__main__":
    main()
