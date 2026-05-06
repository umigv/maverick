default:
    @just --list

# ── Environment ────────────────────────────────────────────────────────────────

# Install dependencies and configure the dev environment
setup:
    python3 scripts/setup_environment.py

# Regenerate pyrightconfig.json from packages in src/
generate-pyrightconfig:
    python3 scripts/generate_pyrightconfig.py

# ── Build & Test ───────────────────────────────────────────────────────────────

# Build the entire workspace
build *args:
    colcon build {{args}}

# Run all tests
test *args:
    colcon test {{args}}
    colcon test-result --verbose

# Build a single package and its dependencies
build-pkg pkg:
    colcon build --packages-up-to {{pkg}}

# Run tests for a single package
test-pkg pkg:
    colcon test --packages-select {{pkg}}
    colcon test-result --verbose

# Delete build, install, and log directories
clean:
    rm -rf build install log

# ── Code Quality ───────────────────────────────────────────────────────────────

# Auto-format all source files
format *args:
    python3 scripts/format.py {{args}}

# Run all linters
lint *args:
    python3 scripts/lint.py {{args}}

# ── Packages ───────────────────────────────────────────────────────────────────

# Create a new ROS 2 package (type: python|cpp)
new dir pkg type='python':
    python3 scripts/create_package.py {{dir}} {{pkg}} --type {{type}}
    just generate-pyrightconfig

# ── Devices ────────────────────────────────────────────────────────────────────

# Create a udev alias for a device
alias device name:
    python3 scripts/create_device_alias.py {{device}} {{name}}

# Remove a udev device alias
unalias name:
    python3 scripts/remove_device_alias.py {{name}}

# Calibrate the ODrive motor controllers
calibrate-odrive:
    python3 scripts/calibrate_odrive.py

# Clear ODrive motor controller errors
clear-odrive-errors:
    python3 scripts/clear_odrive_errors.py

# ── ROS 2 ──────────────────────────────────────────────────────────────────────

# Capture a single message from a topic to a file
extract topic output:
    python3 scripts/extract_message.py {{topic}} {{output}}

# Publish a message to a topic from a file
publish topic input rate='once':
    python3 scripts/publish_message.py {{topic}} {{input}} {{rate}}
