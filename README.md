# UMARV Maverick 2025-2026

## Warning: Intellisense
Always build with `--symlink-install`. Without it, control-clicking a dependency in VSCode will take you to a copy
instead of the source file, and data files (launch files, config, course JSON) written to the install directory will
not reflect back to the source tree. For example:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Dependencies
You can install all dependencies by running
```bash
./scripts/setup.sh
```

## Running the stack
Run each of these commands in separate terminals:
```bash
ros2 launch bringup base.launch.py mode:=<mode> [simulation:=true] [course:=<course>]
```
```bash
ros2 launch bringup teleop.launch.py controller:=<ps4/xbox>
```
and / or
```bash
ros2 launch bringup navigation.launch.py mode:=<mode> [course:=<course>]
```

### Mode and Course Configuration
See [bringup/README.md](bringup/README.md) for mode and course configuration 
details.


### Visualization
Run in a separate terminal:
```bash
ros2 launch bringup visualization.launch.py
```
This sends robot data to Foxglove. Then open [Foxglove Studio](https://foxglove.dev/download) and connect to 
`ws://localhost:8765`.

Alternatively, run `rviz2` in a new terminal and add the topics you want to visualize.
