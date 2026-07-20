# Development

How to work in this codebase: environment setup, the build/test loop, repo structure, and the conventions every change is expected to follow.

## Environment Setup

We offer first class support for:

- System: Linux (x64/arm64), MacOS (Apple Silicon). Windows is supported through WSL2, and MacOS (Intel) support is untested.
- Shell: bash, zsh, fish
- Editor: VSCode. Install recommended extensions in this repo (it should automatically prompt you).

First run the [host bootstrap](https://github.com/umigv/nav-environment) if you haven't. Then:

```bash
just setup
```

Everything - ROS itself, the build toolchain, all dependencies - lives in the pixi environment, which direnv activates automatically inside the repo. Never install ROS system-wide or `source /opt/ros/...`. Mixing a system ROS into the pixi environment breaks in confusing ways (wrong Python, broken `rclpy` imports).

## Tooling

All workflows go through `just` recipes, which run inside the pixi environment via `pixi run`, so they work even in a shell where the environment isn't activated. Don't invoke the scripts in `scripts/` directly.

Run bare `just` to list every recipe. The core workflows:

```bash
just build                # Build the workspace
just build-pkg <package>  # Build one package and its dependencies
just test                 # Run all tests
just test-pkg <package>   # Run tests for one package
just lint                 # Check formatting and lint
just format               # Auto-fix formatting
just clean                # Delete build/install/log
```

## Repo Structure

| Directory           | Contents                                                                                |
| ------------------- | --------------------------------------------------------------------------------------- |
| `src/bringup`       | Launch files, mode/course configs, and the top-level entry points for running the stack |
| `src/core`          | Shared messages and library code used across packages                                   |
| `src/description`   | URDFs and robot/world description packages                                              |
| `src/hardware`      | Drivers for onboard hardware                                                            |
| `src/localization`  | Odometry and coordinate-frame conversion packages                                       |
| `src/navigation`    | Path planning, path tracking, mission control, and recovery behavior packages           |
| `src/simulation`    | Simulated sensors and environment for testing without hardware                          |
| `src/visualization` | Visualization packages                                                                  |
| `src/template`      | Package skeletons copied by `just create-package`                                       |

Each package documents its own interface (topics, services, behavior) in its README. [bringup/README.md](../src/bringup/README.md) documents the stack-wide wiring between them.

## Where to Add Dependencies

All dependencies are installed by pixi and declared in `pyproject.toml`:

| Dependency                                       | Where it goes                                                                  |
| ------------------------------------------------ | ------------------------------------------------------------------------------ |
| ROS package used by a node                       | `[tool.pixi.feature.ros.dependencies]` as `ros-<distro>-*` (robostack channel) |
| Python or C/C++ library available on conda-forge | `[tool.pixi.feature.ros.dependencies]`                                         |
| Python library only on PyPI                      | `[tool.pixi.feature.ros.pypi-dependencies]`                                    |
| Repo-wide dev/lint tooling                       | `[tool.pixi.feature.tooling.dependencies]`                                     |

`package.xml` only requires dependencies on other packages in this workspace - colcon reads it for build ordering. External dependencies don't need a `package.xml` entry.

After changing dependencies, `pixi.lock` should change to reflect edited dependencies. It is part of the change - commit it. Never add a `requirements.txt` or install anything with apt/pip by hand: the environment must stay fully described by `pyproject.toml` + `pixi.lock` so it is reproducible on every machine and in CI.

The ROS distro is named only in `pyproject.toml` (the `ros-<distro>-*` dependency names). Never hardcode the distro anywhere else - a distro bump should touch only `pyproject.toml` and `pixi.lock`.

## Creating a Package

```bash
just create-package <dir> <package> [--type python|cpp]
```

Copies [`template_python`](../src/template/template_python) or [`template_cpp`](../src/template/template_cpp) into `<dir>/<package>`. Fill out every `TODO` the template leaves.

## Node Configuration

- First-party nodes take no YAML files.
- Parameters live in a frozen config dataclass loaded via `utils.config`, with validation in `__post_init__` - the dataclass and its docstring are the parameter documentation. See the [utils README](../src/core/utils/README.md#utilsconfig) for the loader semantics.
- Parameters shared with other nodes (e.g. frame IDs, GPS datum and course file paths, e-stop file path) are required and never defaulted. They are to be injected by launch files to ensure no drift.

## Documentation

Beyond what the linters enforce and what's in the template README instructions:

- Update documentation in the same PR as the behavior it documents.
- Only C++ packages get a `Config Parameters` table as they have no dataclasses with all the configs documented together.
- Use a plain hyphen surrounded by spaces as the separator, never an em-dash. Bullet descriptions after the separator start capitalized.
- Topic bullets follow `` `topic` (`pkg/Msg`) - Description `` with the `msg`/`srv` segment omitted from type names.

## Code Style

Beyond what the linters enforce:

- Numeric names carry unit suffixes: `_m`, `_s`, `_mps`, `_radps`, `_m2`, and so on (e.g. `waypoint_reached_threshold_m`, `control_period_s`).
- Recurring timing is expressed as a period in seconds (`publish_period_s`), never as a rate or frequency in Hz.
- C++ #includes: `<...>` for external dependencies, `"..."` for the package's own headers.
- No em-dash and no `“`. Unfortunately they are too AI-coded nowadays.

## Cross-Cutting Conventions

- TF frame names are defined once in `bringup/config/frames.yaml`.
- Nodes publish and subscribe generic topic names (`odom`, `occupancy_grid`, `goal`). Launch files remap them to the stack-wide wiring documented in [bringup/README.md](../src/bringup/README.md). Never hardcode a stack-specific topic path in a node.
- Latched topics (e.g. `mission_state`, ground-truth maps) must use `utils.qos.LATCHED` on both the publisher and subscriber - a QoS mismatch silently drops all messages.
- The e-stop state is a file, not a topic: `estop_driver` writes it, other nodes read it directly. See [estop_driver](../src/hardware/estop_driver/README.md).
