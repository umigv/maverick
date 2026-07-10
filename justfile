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
build-pkg pkg *args:
    colcon build --packages-up-to {{pkg}} {{args}}

# Run tests for a single package
test-pkg pkg *args:
    colcon test --packages-select {{pkg}} {{args}}
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
create-package dir package type='python':
    python3 scripts/create_package.py {{dir}} {{package}} --type {{type}}
    just generate-pyrightconfig

# ── Devices ────────────────────────────────────────────────────────────────────

# Create a udev alias for a device
alias device name:
    python3 scripts/create_device_alias.py {{device}} {{name}}

# Remove a udev device alias
unalias name:
    python3 scripts/remove_device_alias.py {{name}}

# ── Waypoints ──────────────────────────────────────────────────────────────────

# Convert all packed-DMS waypoints (waypoints/dms/) to decimal degrees (waypoints/decimal/)
convert-waypoints:
    python3 scripts/convert_waypoint.py

# ── ROS 2 ──────────────────────────────────────────────────────────────────────

# Capture a single message from a topic to a file
extract topic output:
    python3 scripts/extract_message.py {{topic}} {{output}}

# Publish a message to a topic from a file
publish topic input rate='once':
    python3 scripts/publish_message.py {{topic}} {{input}} {{rate}}

# ── ODrive ────────────────────────────────────────────────────────────────────

# Calibrate the ODrive motor controllers
calibrate-odrive:
    python3 src/hardware/odrive_driver/scripts/calibrate_odrive.py

# Clear ODrive motor controller errors
clear-odrive-errors:
    python3 src/hardware/odrive_driver/scripts/clear_odrive_errors.py

# Plot ODrive Iq and velocity (requires odrive_driver running with debug: true)
plot-odrive *args:
    python3 src/hardware/odrive_driver/scripts/plot_motor_signals.py {{args}}

# ── VectorNav ──────────────────────────────────────────────────────────────────

# Monitor IMU orientation as Euler angles
euler-monitor *args:
    python3 src/hardware/vectornav_driver/scripts/euler_monitor.py {{args}}

# Monitor VectorNav INS/GNSS status
vectornav-monitor:
    python3 src/hardware/vectornav_driver/scripts/vectornav_monitor.py
