# TODO: move into bringup README, update requirements for cuda, establish specific zed sdk requirement

# zed_driver

Integration of the Stereolabs [ZED ROS 2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) for the two ZED cameras.

- `bringup/launch/zed.launch.py` - starts both cameras
- `bringup/config/hardware/zed.yaml` - enabling/disabling topics

## Requirements

- ZED SDK installed at `/usr/local/zed`

## Launching

The cameras are started as part of the hardware bringup. To run only the cameras:

```bash
ros2 launch bringup zed.launch.py left_serial:=<serial> right_serial:=<serial>
```

| Argument       | Default    | Description                                                     |
| -------------- | ---------- | --------------------------------------------------------------- |
| `left_serial`  | `39394535`   | Serial number of the left camera                                |
| `right_serial` | `36466710` | Serial number of the right camera                               |

## Published Topics

Each camera publishes under `/zed/<camera_name>/` (`<camera_name>` is `zed_left` or `zed_right`):

- `rgb/color/rect/image` (`sensor_msgs/Image`) - Color image, `bgra8`
- `rgb/color/rect/camera_info` (`sensor_msgs/CameraInfo`) - Intrinsics for the RGB image
- `depth/depth_registered` (`sensor_msgs/Image`) - Depth in meters, `32FC1` format (1 channel, each pixel is an `f32`)
- `depth/camera_info` (`sensor_msgs/CameraInfo`) - Intrinsics for the depth image

## Configuration

`bringup/config/hardware/zed.yaml` applies to both cameras:

| Parameter                         | Value          | Notes                                                                         |
| --------------------------------- | -------------- | ----------------------------------------------------------------------------- |
| `general.grab_resolution`         | `HD720`        | `HD2K`, `HD1080`, `HD720`, `VGA`                                              |
| `general.grab_frame_rate`         | `30`           | // TODO: double check this                                                    |
| `general.pub_resolution`          | `NATIVE`       |                                                                               |
| `depth.depth_mode`                | `NEURAL_LIGHT` |                                                                               |

`publish_*` flags only advertise topics.

## Updating the Wrapper

To move to a new release of the ros wrapper after upgrading the SDK:

```bash
git -C src/hardware/zed_driver/third_party/zed-ros2-wrapper fetch --tags
git -C src/hardware/zed_driver/third_party/zed-ros2-wrapper checkout <tag>
git add src/hardware/zed_driver/third_party/zed-ros2-wrapper
```

## Troubleshooting

- **Second camera fails to open** - both serials are `0` or the same; set `left_serial`/`right_serial`
