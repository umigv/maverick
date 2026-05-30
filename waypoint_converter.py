import re


def dms_packed_to_decimal(value):
    sign = -1 if value < 0 else 1
    value = abs(value)

    d = int(value)
    remainder = round((value - d) * 100, 10)
    m = int(remainder)
    s = round((remainder - m) * 100, 10)

    decimal = sign * (d + m / 60 + s / 3600)

    print(f"    [parse] raw={value:.9f} → d={d}° m={m}' s={s:.7f}\" → decimal={decimal:.9f}")
    return decimal


def parse_line(line):
    line = line.strip()
    if not line:
        return None

    print(f"  [line] {line!r}")

    match = re.match(r"^(.+?)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)$", line)
    if not match:
        raise ValueError(f"Could not parse line: {line!r}")

    name, lat_str, lon_str = match.group(1), match.group(2), match.group(3)
    print(f"  [line] name={name!r}  lat_str={lat_str!r}  lon_str={lon_str!r}")

    print("  [line] parsing lat:")
    lat = dms_packed_to_decimal(float(lat_str))
    print("  [line] parsing lon:")
    lon = dms_packed_to_decimal(float(lon_str))

    return name, lat, lon


def convert(text):
    results = []
    lines = text.strip().splitlines()
    print(f"[convert] {len(lines)} line(s) to process")
    for i, line in enumerate(lines):
        print(f"[convert] line {i + 1}:")
        parsed = parse_line(line)
        if parsed:
            name, lat, lon = parsed
            print(f"[convert] line {i + 1} result: {name},{lat:.9f},{lon:.9f}")
            results.append(parsed)
    return results


# ── Example usage ──────────────────────────────────────────────────────────────

sample = """5001 North, 42.400577100, -83.130962509
5006 North Mid, 42.400523432, -83.130969819
5005 South Mid, 42.400507588, -83.130969297
5000 South, 42.400453974, -83.130957951
5002 North, 42.400556255, -83.130645144
5004 Mid, 42.400510946, -83.130640432
5003 South, 42.400465621, -83.130635756"""

print("=== conversion ===")
for name, lat, lon in convert(sample):
    print(f"{name},{lat:.9f},{lon:.9f}")
