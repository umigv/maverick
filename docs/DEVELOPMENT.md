# Development

How to work in this codebase: environment setup, the build/test loop, repo structure, and the conventions every change is expected to follow. For how changes get merged (branches, PRs, CI), see [CONTRIBUTING.md](CONTRIBUTING.md). For running the robot, see the [README](../README.md).

## Environment Setup

Linux (x64/arm64) and macOS (Apple silicon) are supported. First run the [host bootstrap](https://github.com/umigv/nav-environment) if you haven't - it provides the two system tools this repo expects (`just` and `direnv`). Then:

```bash
just setup
```

Everything else - ROS, the build toolchain, and all workspace dependencies - lives in the pixi environment, defined in `pyproject.toml` (`[tool.pixi.*]`) with exact versions pinned in `pixi.lock`. The environment is created and kept up to date automatically whenever something runs through `pixi run`; there is no apt, rosdep, or pip step. Setup itself is idempotent and performs:

- Initializes git submodules (and clears `build`/`install`/`log` if they changed)
- Allows direnv in the repo: entering the directory auto-activates the pixi environment (`.envrc` runs `pixi shell-hook`), so raw `ros2` and `colcon` commands work in any shell inside the repo
- Installs `ros2` CLI completions for your shell (bash, zsh, or fish)
- Generates `pyrightconfig.json`
- Installs git hooks: `pre-commit` runs lint; `post-checkout`/`post-merge`/`post-rewrite` regenerate `pyrightconfig.json`

Open a new terminal afterwards for the shell changes to take effect. VS Code: install the recommended extensions when prompted.

## The Core Loop

All workflows go through `just` - run bare `just` to list every recipe. Recipes run inside the pixi environment via `pixi run`, so they work even in a shell where the environment isn't activated. The scripts in `scripts/` are implementation details of the recipes; don't invoke them directly.

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

| Dependency                                       | Where it goes                                                                                                                                                        |
| ------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ROS package used by a node                       | `[tool.pixi.feature.ros.dependencies]` in `pyproject.toml` as `ros-<distro>-*` (robostack channel), plus a `<depend>`/`<exec_depend>` in the package's `package.xml` |
| Python or C/C++ library available on conda-forge | `[tool.pixi.feature.ros.dependencies]`, plus the `package.xml` entry                                                                                                 |
| Python library only on PyPI                      | `[tool.pixi.feature.ros.pypi-dependencies]`, plus a `-pip`-suffixed `package.xml` entry (e.g. `python3-odrive-pip`)                                                  |
| Repo-wide dev/lint tooling                       | `[tool.pixi.feature.tooling.dependencies]`                                                                                                                           |

pixi is what installs dependencies; `package.xml` still declares what each package uses (colcon reads it for build ordering, and it keeps per-package usage auditable). After changing dependencies, the refreshed `pixi.lock` is part of the change - commit it. Never add a `requirements.txt` or install anything with apt/pip by hand: the environment must stay fully described by `pyproject.toml` + `pixi.lock` so it is reproducible on every machine and in CI.

## Creating a Package

```bash
just create-package <dir> <package> [--type python|cpp]
```

Copies [`template_python`](../src/template/template_python) or [`template_cpp`](../src/template/template_cpp) into `<dir>/<package>`: a README stub, a config dataclass (Python), and the standard `setup.py`/`CMakeLists.txt`/`package.xml` shape.

Config convention: first-party nodes take no YAML files. Parameters live in a frozen config dataclass loaded via `utils.config`, with validation in `__post_init__` - the dataclass and its docstring are the parameter documentation. See the [utils README](../src/core/utils/README.md#utilsconfig) for the loader semantics. Launch files inject cross-cutting values (frame IDs, file paths) as inline parameters.

## Code Style

- `just lint` runs ruff (format check + lint), mypy, clang-format, and mdformat; `just format` auto-fixes formatting. All are configured in `pyproject.toml`.
- The whole lint toolchain is pinned by pixi (e.g. `clang-format = "22.*"`), so local results always match CI - there is no version drift to debug.
- C++ includes: `<...>` for external dependencies, `"..."` for the package's own headers.

## Cross-Cutting Conventions

- TF frame names are defined once in `bringup/config/frames.yaml` and injected into nodes by launch files - never hardcode a frame name in a node.
- Latched topics (`mission_state`, ground-truth maps) must use `utils.qos.LATCHED` on both the publisher and subscriber - a QoS mismatch silently drops all messages.
- The e-stop state is a file, not a topic: `estop_driver` writes it, other nodes read it directly. See [estop_driver](../src/hardware/estop_driver/README.md).
