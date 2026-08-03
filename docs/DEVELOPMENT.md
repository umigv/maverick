# Development

How to work in this codebase: environment setup, the build/test loop, repo structure, and the conventions changes should follow.

## Environment Setup

We offer first class support for:

- System: Linux (x64/arm64), macOS (Apple Silicon), and Windows (through WSL2)
- Shell: bash, zsh, fish
- Editor: VSCode

First run the [host bootstrap](https://github.com/umigv/nav-environment) if you haven't.

> [!WARNING]
> Do not proceed without having run the bootstrap script.

Then:

```bash
just setup
```

Also be sure to install the recommended extensions of this repo.

Everything: ROS, the build toolchain, and dependencies, lives in the pixi environment. This environment is automatically activated by [direnv](https://direnv.net/) when you're within the repo.

> [!WARNING]
> Never install ROS system-wide or `source /opt/ros/...`. Mixing a system ROS into the pixi environment breaks in confusing ways (wrong Python, broken `rclpy` imports).

## Tooling

All workflows go through [`just`](https://just.systems/man/en/) recipes.

> [!WARNING]
> Don't invoke the scripts in `scripts/` directly. Always use `just`.

### Common Recipes

Run `just` (without arguments) to list every recipe. The core workflows:

```bash
just build                # Build the workspace
just build-package <package>  # Build one package and its dependencies
just test                 # Run all tests
just test-package <package>   # Run tests for one package
just lint                 # Check formatting and lint
just format               # Auto-fix formatting
just clean                # Delete build/install/log
```

## Repo Structure

| Directory           | Contents                                                                                |
| ------------------- | --------------------------------------------------------------------------------------- |
| `src/bringup`       | Launch files, mode/course configs, and the top-level entry points for running the stack |
| `src/core`          | Shared messages and library code used across packages                                   |
| `src/cv`            | Computer vision algorithms                                                              |
| `src/description`   | URDFs and robot/world description packages                                              |
| `src/hardware`      | Drivers for onboard hardware                                                            |
| `src/localization`  | Odometry and coordinate-frame conversion packages                                       |
| `src/navigation`    | Path planning, path tracking, mission control, and recovery behavior packages           |
| `src/simulation`    | Simulated sensors and environment for testing without hardware                          |
| `src/visualization` | Visualization packages                                                                  |
| `src/template`      | Package skeletons copied by `just create-package`                                       |

Each package documents its topics, services, and behavior in its README. [bringup/README.md](../src/bringup/README.md) documents the stack-wide wiring between them.

## Where to Add Dependencies

All dependencies are installed by pixi and declared in `pyproject.toml`. Pick the section by who needs it (row) and where it's published (column):

|                                      | Available on conda-forge                   | Only on PyPI                                    |
| ------------------------------------ | ------------------------------------------ | ----------------------------------------------- |
| Robot code (`ros` feature)           | `[tool.pixi.feature.ros.dependencies]`     | `[tool.pixi.feature.ros.pypi-dependencies]`     |
| Dev/lint tooling (`tooling` feature) | `[tool.pixi.feature.tooling.dependencies]` | `[tool.pixi.feature.tooling.pypi-dependencies]` |

ROS packages come from the robostack channel and are named `ros-<distro>-*`; they always go in `[tool.pixi.feature.ros.dependencies]`.

> [!WARNING]
> Never add a `requirements.txt` or install anything with apt / pip by hand. The environment must stay fully described by `pyproject.toml` + `pixi.lock` so it is reproducible on every machine and in CI.

After changing dependencies, `pixi.lock` should change to reflect edited dependencies, make sure to commit it!

The ROS distro is named only in `pyproject.toml` (the `ros-<distro>-*` dependency names). Never hardcode the distro anywhere else - a distro bump should touch only `pyproject.toml` and `pixi.lock`.

In a standard ROS2 workspace, a package's `package.xml` must declare all its dependencies. This is because standard ROS2 workspaces use it to:

1. Install external dependencies using [rosdep](https://docs.ros.org/en/rolling/ROS-Framework/client-libraries/Working-with-Client-Libraries/Rosdep.html)
2. Determine build order of packages in this workspace

As we use pixi to manage external dependencies, only declare dependencies on other packages in this workspace (`<exec_depend>` in Python packages, `<depend>` in C++ packages).

## Creating a Package

```bash
just create-package <dir> <package> [--type python|cpp]
```

Copies [`template_python`](../src/template/template_python) or [`template_cpp`](../src/template/template_cpp) into `<dir>/<package>`. The new files will contain TODO statements, which you should resolve and delete.

## Node Configuration

How a node declares its parameters depends on the kind of node:

- Third-party nodes - a YAML file in `bringup/config/`.
- First-party C++ nodes - a YAML file in `bringup/config/`, with every parameter documented in a `Config Parameters` table in the package README (no `utils.config` equivalent exists for C++ yet).
- First-party Python nodes - no YAML. Parameters live in a frozen config dataclass loaded via `utils.config`:
  - Defaults live in the dataclass fields.
  - Validation goes in `__post_init__`.
  - Every parameter is documented in the dataclass docstring.
  - See the [utils README](../src/core/utils/README.md#utilsconfig) for the loader semantics.

For all nodes, parameters shared with other nodes (e.g. frame IDs, GPS datum and course file paths, e-stop file path) are never defaulted and never written into a YAML. Launch files inject them so the values can't drift between nodes.

## Documentation

READMEs follow the section vocabulary, ordering, and formats shown in the template READMEs ([python](../src/template/template_python/README.md), [cpp](../src/template/template_cpp/README.md)).

Beyond that:

- Update documentation in the same PR as the behavior it documents.
- Type names omit the `msg`/`srv` segment: `nav_msgs/Odometry`, not `nav_msgs/msg/Odometry`.

## Code Style

Beyond what the linters enforce:

- Avoid abbreviations unless it's an established convention. One that's obvious to you may not be obvious to others.
- Numeric names carry SI unit suffixes: `_m`, `_s`, `_mps`, `_radps`, `_m2`, and so on (e.g. `waypoint_reached_threshold_m`, `control_period_s`).
- Recurring timing is expressed as a period in seconds (`publish_period_s`), never as a rate or frequency in Hz.
- No unicode characters. Use ASCII equivalents e.g. `->` for arrows, `^2` for superscripts, `deg` for degrees, and spelled-out names (`pi`, `theta`, `omega`) for Greek letters.
  - AI likes to use unicode characters, so please keep an eye out if you're using an AI tool!

## Cross-Cutting Conventions

- TF frame names are defined once in `bringup/config/frames.yaml`.
- Nodes publish and subscribe generic topic names (`odom`, `occupancy_grid`, `goal`). Launch files remap them to the stack-wide wiring documented in [bringup/README.md](../src/bringup/README.md). Never hardcode a stack-specific topic path in a node.
  - For example, `vectornav_driver` publishes generic `vectornav/odom`; `hardware.launch.py` remaps it to the stack-wide name with `remappings=[("vectornav/odom", "odom/global")]`.
- Latched topics (e.g. `mission_state`, ground-truth maps) must use `utils.qos.LATCHED` on both the publisher and subscriber.

> [!WARNING]
> A QoS mismatch silently drops all messages.
