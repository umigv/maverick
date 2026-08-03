#!/bin/bash

# define environment names and requirement files
ENV1="base_env"
REQ1="requirements_base.txt"

ENV2="hsv_env"
REQ1="requirements_hsv.txt"

echo "Starting environment setup"

# function to create and install
setup_venv() {
    local name=$1
    local reqs=$2

    echo "Creating virtual environment: $name"
    python3 -m venv "$name"

    echo "Activating and installing $reqs..."
    source "$name/bin/activate"

    if [ -f "$reqs" ]; then
        pip install --upgrade pip
        pip install -r "$reqs"
    else
        echo "Warning: $reqs not found. Skipping installation"
    fi

    deactivate
    echo "$name is ready!"
}

# run setups
setup_venv "$ENV1" "$REQ1"
setup_venv "$ENV2" "$REQ2"

echo "All environments have been created successfully"