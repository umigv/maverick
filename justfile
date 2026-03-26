default:
    @just --list

build *args:
    colcon build {{args}}

test *args:
    colcon test {{args}}
    colcon test-result --verbose

lint *args:
    scripts/format-lint.sh {{args}}

check *args:
    scripts/checks.sh {{args}}

alias device name:
    scripts/create-device-alias.sh {{device}} {{name}}

unalias name:
    scripts/remove-device-alias.sh {{name}}

new-pkg dir pkg:
    cd {{dir}} && {{justfile_directory()}}/scripts/create-package.sh {{pkg}}

setup:
    scripts/setup.sh

build-pkg pkg:
    colcon build --packages-up-to {{pkg}}

test-pkg pkg:
    colcon test --packages-select {{pkg}}
    colcon test-result --verbose

clean:
    rm -rf build install log

extract topic output:
    #!/usr/bin/env bash
    set -euo pipefail

    type="$(ros2 topic type "{{topic}}")"
    {
        echo "# ros2_type: ${type}"
        ros2 topic echo --once --full-length "{{topic}}"
    } > "{{output}}"

publish topic input rate='once':
    #!/usr/bin/env bash
    set -euo pipefail

    type="$(sed -n 's/^# ros2_type:[[:space:]]*//p' "{{input}}" | head -n1)"
    payload="$(sed -e '1{/^# ros2_type:[[:space:]]*/d;}' -e '/^---$/d' "{{input}}")"

    if [[ -z "${type}" ]]; then
        echo "Error: could not find '# ros2_type: ...' header in {{input}}" >&2
        exit 1
    fi

    if [[ "{{rate}}" == "once" ]]; then
        ros2 topic pub --once "{{topic}}" "${type}" "${payload}"
    else
        ros2 topic pub -r "{{rate}}" "{{topic}}" "${type}" "${payload}"
    fi
