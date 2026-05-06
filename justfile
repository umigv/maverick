default:
    @just --list

build *args:
    colcon build {{args}}

test *args:
    colcon test {{args}}
    colcon test-result --verbose

format *args:
    python3 scripts/format.py {{args}}

lint *args:
    python3 scripts/lint.py {{args}}

alias device name:
    python3 scripts/create_device_alias.py {{device}} {{name}}

unalias name:
    python3 scripts/remove_device_alias.py {{name}}

### type: python | cpp
new dir pkg type='python':
    cd {{dir}} && python3 {{justfile_directory()}}/scripts/create_package.py {{pkg}} --type {{type}}

setup:
    python3 scripts/setup_environment.py

build-pkg pkg:
    colcon build --packages-up-to {{pkg}}

test-pkg pkg:
    colcon test --packages-select {{pkg}}
    colcon test-result --verbose

calibrate-odrive:
    python3 scripts/calibrate_odrive.py

clear-odrive-errors:
    python3 scripts/clear_odrive_errors.py

clean:
    rm -rf build install log

extract topic output:
    python3 scripts/extract_message.py {{topic}} {{output}}

publish topic input rate='once':
    python3 scripts/publish_message.py {{topic}} {{input}} {{rate}}
