# UMARV Maverick 2025-2026
![Maverick](docs/images/maverick.jpg)

## `src/` Layout
| Category | Contents |
|---|---|
| `bringup` | Launch files, mode/course configs, and the top-level entry points for running the stack |
| `core` | Shared messages and library code used across packages |
| `description` | URDFs and robot/world description packages |
| `hardware` | Drivers for onboard hardware |
| `localization` | Odometry and coordinate-frame conversion packages |
| `navigation` | Path planning, path tracking, mission control, and recovery behavior packages |
| `simulation` | Simulated sensors and environment for testing without hardware |
| `visualization` | Visualization packages |
| `template` | Package skeletons copied by `just create-package` |

## Setup
First run the [host bootstrap](https://github.com/umigv/nav-environment) if you haven't. Then:
```bash
./scripts/setup_environment.py
```

VSCode: Install recommended extensions in this repo (it should automatically prompt you).

## Commands
```bash
# See available commands
just
```

## Running the Stack
```bash
# Build the workspace
just build

# Build a single package including dependencies
just build-pkg <package>
```

Run each of these commands in separate terminals:
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

### Mode and Course Configuration
See [bringup/README.md](bringup/README.md) for mode and course configuration details.

### Visualization
Run in a separate terminal:
```bash
ros2 launch bringup visualization.launch.py
```
This sends robot data to Foxglove. Then open [Foxglove Studio](https://foxglove.dev/download) and connect to
`ws://localhost:8765`.

Alternatively, run `rviz2` in a new terminal and add the topics you want to visualize.

## Testing
```bash
# Run all tests
just test

# Run tests for a single package
just test-pkg <package>
```

## Formatting & Linting
```bash
# Check formatting and lint
just lint

# Auto-fix formatting
just format
```

## Adding a New Package
```bash
just create-package <dir> <package> [--type python|cpp]
```
Copies [`template_python`](src/template/template_python) or [`template_cpp`](src/template/template_cpp) into
`<dir>/<package>`.
