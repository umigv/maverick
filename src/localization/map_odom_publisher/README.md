# map_odom_publisher
Computes and broadcasts the `map_frame_id`→`odom_frame_id` TF by combining global odometry with the local TF tree.

Caches the latest `odom` message and recomputes on a timer at `publish_period_s`. On each tick, looks up `odom_frame_id`
→`base_frame_id` from TF and computes:
```
T_map_odom = T_map_base * T_odom_base^-1
```

## Subscribed Topics
- `odom` (`nav_msgs/Odometry`) - Global odometry. `frame_id` must be `map_frame_id` and `child_frame_id` must be 
`base_frame_id`

## Broadcasted TF Frames
- `map_frame_id` → `odom_frame_id`

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `map_frame_id` | `map` | Expected `frame_id` of `odom` and parent of the broadcasted TF |
| `odom_frame_id` | `odom` | Child frame of the broadcasted TF; used for TF lookup |
| `base_frame_id` | `base_link` | Expected `child_frame_id` of `odom`; used for TF lookup |
| `publish_period_s` | `0.01` | Timer period in seconds for broadcasting the TF |
