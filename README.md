# UMARV Maverick 2025-2026

## Dependencies
You can install all dependencies by running
```bash
./scripts/setup_environment.py
```

## Running the stack
To build the stack, run 
```
just build
```

Run each of these commands in separate terminals:
```bash
ros2 launch bringup base.launch.py mode:=<mode> [simulation:=true] [course:=<course>]
```
```bash
ros2 launch bringup teleop.launch.py controller:=<xbox/xbox_wireless/ps4>
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
