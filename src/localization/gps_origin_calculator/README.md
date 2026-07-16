# gps_origin_calculator

Collects GPS samples on startup, then computes and writes the median position (latitude, longitude, altitude) as the datum.

Samples are filtered by horizontal accuracy (`max_horizontal_stdev_m`). The node waits until either `min_samples_required` samples have been collected for at least `min_sample_duration_s`, or `max_sample_duration_s` has elapsed.

If `output_file` is set, the computed datum is written directly into the specified `gps.json`, overwriting only the `datum` field and preserving existing waypoints.

## Subscribed Topics

- `gps` (`sensor_msgs/NavSatFix`) - GPS fix
