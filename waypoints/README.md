# Waypoints

GPS waypoints for IGVC courses. The competition hands out coordinates in a **packed DMS** format; we store those verbatim as the source of truth and generate decimal-degree versions on demand for tools (e.g. Google My Maps, `bringup/courses`) that expect them.

## Layout

```
waypoints/
  dms/                      # source of truth
    my_waypoint.csv
  decimal/                  # generated - decimal degrees, do not edit
    my_waypoint.csv
```

- **`dms/`** is what you edit. These hold the coordinates exactly as the competition provides them. Add a new course by dropping a CSV in here.
- **`decimal/`** is generated from `dms/` by the converter. Treat it as a build artifact: never edit it by hand, just regenerate.

Files in the two folders share a basename (`igvc2026_autonav.csv` -> `igvc2026_autonav.csv`), so a `dms/` file always maps to the `decimal/` file of the same name.

## CSV format

Both files have a `name,latitude,longitude` header row, one waypoint per line:

```
name,latitude,longitude
5001 North,42.400577100,-83.130962509
```

In general, decimal latitude and longitude should be written to 7 decimal places (and 2 for altitude):

1. It's around 1cm precision which is finer than what we can measure
2. Doesn't match the 9dp precision of the data IGVC provides to prevent confusion

This should be kept in sync across all producers / consumers of GPS coordinates across ARV (i.e. `gps_origin_calculator`, `bringup/courses` `gps.json` files, course creation tool).

## Packed DMS format

The `latitude`/`longitude` numbers in `dms/` are **not** decimal degrees. They pack degrees, minutes, and seconds into a single float as `DD.MMSSsssss`.

```
42.400577100  ==  42 deg 40 min 05.771 sec
```

Converted to decimal degrees that's `42 + 40/60 + 5.771/3600 = 42.6682697`, which is what lands in `decimal/`.

## Generating the decimal files

Run the converter:

```sh
just convert-waypoints
```

It converts every `*.csv` in `dms/`, writing the matching file into `decimal/`, and prints how many waypoints each one had. Re-run it whenever you change anything in `dms/` so `decimal/` stays in sync.

## Visualizing

To sanity-check that the converted coordinates land where you expect, import a `decimal/` CSV into [Google My Maps](https://mymaps.google.com): create a new map, **Import** the CSV as a layer, pick `latitude`/`longitude` for positioning and `name` for the marker titles. The waypoints should appear on the course in the right spots.
