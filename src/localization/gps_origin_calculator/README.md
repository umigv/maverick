# gps_origin_calculator
Collects GPS samples on startup, then computes and writes the median position (latitude, longitude, altitude) as the
datum. See `bringup` for the launch file that runs this alongside the GPS driver.

Samples are filtered by horizontal accuracy (`max_horizontal_stdev_m`). The node waits until either `min_samples_required`
samples have been collected for at least `min_sample_duration_s`, or `max_sample_duration_s` has elapsed.

If `output_file` is set, the computed datum is written directly into the specified `gps.json`, overwriting only the
`datum` field and preserving existing waypoints.

## Subscribed Topics
- `gps` (`sensor_msgs/NavSatFix`) - GPS fix

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `min_samples_required` | `3000` | Minimum number of valid GPS samples before setting datum |
| `min_sample_duration_s` | `60.0` | Minimum time (s) to collect samples |
| `max_sample_duration_s` | `90.0` | Maximum time (s) before setting datum regardless of sample count |
| `max_horizontal_stdev_m` | `1.0` | Maximum horizontal accuracy (m) for a sample to be accepted |
| `output_file` | `""` | Path to `gps.json` to write the computed datum into. Empty string skips writing |
