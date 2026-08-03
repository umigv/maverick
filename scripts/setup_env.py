import os
import subprocess
import sys

ENV1 = "base_env"
REQ1 = "requirements_base.txt"

ENV2 = "hsv_env"
REQ2 = "requirements_hsv.txt"

def run_command(cmd: str, env=None):
    try:
        subprocess.check_call(cmd, shell=True, env=env)
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {cmd}")
        sys.exit(e.returncode)

def setup_venv(name: str, reqs: str):
    print(f"Creating virtual environment: {name}")
    run_command(f"python3 -m venv {name}")

    pip_path = os.path.join(name, "bin", "pip")

    print(f"Installing dependencies from {reqs}...")
    if os.path.isfile(reqs):
        run_command(f"{pip_path} install --upgrade pip")
        run_command(f"{pip_path} install -r {reqs}")
    else:
        print(f"Warning: {reqs} not found. Skipping installation")

    print(f"{name} is ready!\n")

if __name__ == "__main__":
    setup_venv(ENV1, REQ1)
    setup_venv(ENV2, REQ2)