# UMARV Maverick 2025-2026

![Maverick](docs/images/maverick.jpg)

Maverick is UMARV's autonomous ground robot for the 2025-2026 competition season. This repository is its full ROS 2 stack: hardware drivers, localization, navigation, and simulation, launched through a single set of entry points with interchangeable real and simulated sensors.

## Quick Start

For the first time, [set up the environment](docs/DEVELOPMENT.md#environment-setup)

Build the workspace:

```bash
just build
```

Then run the stack, each command in its own terminal. `simulation:=true` runs without hardware:

```bash
ros2 launch bringup base.launch.py mode:=<mode> [simulation:=true] [course:=<course>]
```

```bash
ros2 launch bringup teleop.launch.py controller:=<xbox/xbox_wireless/ps4/ps4_wireless>
```

and / or

```bash
ros2 launch bringup navigation.launch.py mode:=<mode> [course:=<course>]
```

See [bringup/README.md](src/bringup/README.md) for what each mode does and how courses are configured.

## Visualization

```bash
ros2 launch bringup visualization.launch.py
```

This sends robot data to Foxglove. Then open [Foxglove Studio](https://foxglove.dev/download) and connect to `ws://localhost:8765`.

Alternatively, run RViz in a new terminal with the shared stack configuration:

```bash
just rviz
```

See [bringup/README.md](src/bringup/README.md) for which displays are enabled by default. If RViz asks to save the config on exit, decline - saving rewrites the shared config in RViz's verbose format. Keep personal tweaks in your own copy via File > Save Config As.

> **WSL:** set Windows display scaling to a whole-number percentage (e.g. 100%, 200%). Fractional scaling (e.g. 125%, 150%) isn't supported by WSLg and silently falls back to 100%, leaving rviz2's UI illegibly tiny. TODO: See [microsoft/wslg#23](https://github.com/microsoft/wslg/issues/23).

## Tmux

A tmux configuration is included to avoid needing to create too many terminals.

```bash
# Runs rviz and divides terminal into panes in which to run the launch commands
just tmux
```

To exit, close the window by pressing the prefix key (Ctrl + B) and then `&` (Shift + 7), and then confirming by typing `y` and pressing Enter. A convenient cheatsheet for other tmux usage can be found at [tmuxcheatsheet.com](https://tmuxcheatsheet.com).

## Documentation

- [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) - Environment setup, build/test/lint loop, repo structure, dependency, code, and documentation conventions
- [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) - Branches, pull requests, reviews, and CI
- [docs/PLAYBOOK.md](docs/PLAYBOOK.md) - Operating the robot on test and competition days
- [src/bringup/README.md](src/bringup/README.md) - System architecture: launch files, modes, and the stack-wide topic wiring
- Every package documents its own interface and behavior in its README
