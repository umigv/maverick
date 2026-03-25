# lat_lon_converter
Exposes a `fromLL` service that converts GPS latitude/longitude to map-frame ENU points. Drop-in replacement for
the `fromLL` service provided by `robot_localization`'s `navsat_transform_node`, without requiring a full filter.

Uses `pyproj` (topocentric pipeline) for WGS84 → ENU conversion relative to a fixed datum.

## Services
- `fromLL` (`robot_localization/srv/FromLL`) - converts a `geographic_msgs/GeoPoint` (lat, lon, alt) to a
  `geometry_msgs/Point` in the map frame

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `datum` | required | ENU origin as `[latitude (deg), longitude (deg), altitude (m)]` |
