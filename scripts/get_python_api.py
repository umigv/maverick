import argparse
import os
import platform
import re
import subprocess
import sys
from pathlib import Path

import requests

os.environ["PYTHONIOENCODING"] = "utf-8"

ZED_SDK_MAJOR = ""
ZED_SDK_MINOR = ""

PYTHON_MAJOR = ""
PYTHON_MINOR = ""

OS_VERSION = ""
ARCH_VERSION = platform.machine()

whl_platform_str = ""

base_url = "https://download.stereolabs.com/zedsdk/"


def pip_install(
    package: str | Path,
    force_install: bool = False,
    ignore_install: bool = False,
    upgrade: bool = False,
    break_system_packages: bool = False,
) -> int:
    try:
        if break_system_packages:
            os.environ["PIP_BREAK_SYSTEM_PACKAGES"] = "1"

        call_list = [sys.executable, "-m", "pip", "install"]
        if ignore_install:
            call_list.append("--ignore-installed")
        if force_install:
            call_list.append("--force-reinstall")
        if upgrade:
            call_list.append("--upgrade")
        if break_system_packages:
            call_list.append("--break-system-packages")
        call_list.append(str(package))
        err = subprocess.check_call(call_list)
    except Exception:
        err = 1
    return err


def check_valid_file(file_path: str | Path) -> bool:
    r"""
    Check if the specified file is likely a valid .whl (Python wheel) file.

    - Wheel files are ZIP archives, so their first 4 bytes are b'PK\x03\x04'.
    - This function checks that the file exists, is at least 150 KB, and
      starts with the ZIP magic bytes.
    """
    path = Path(file_path)

    try:
        file_size = path.stat().st_size / 1000.0
    except FileNotFoundError:
        file_size = 0

    try:
        with path.open("rb") as f:
            header = f.read(4)
        is_zip = header == b"PK\x03\x04"
    except Exception:
        is_zip = False

    return (file_size > 150) and is_zip


def install_win_dep(
    name: str,
    py_vers: int,
    break_system_packages: bool = False,
) -> None:
    if py_vers < 310:
        package_vers = "3.1.5"
    elif py_vers < 312:
        package_vers = "3.1.6"
    else:
        package_vers = "3.1.9"

    whl_file_name = f"{name}-{package_vers}-cp{py_vers}-cp{py_vers}"
    if py_vers <= 37:
        whl_file_name += "m"
    whl_file_name += "-win_amd64.whl"

    whl_file_url = "https://download.stereolabs.com/py/" + whl_file_name
    print("-> Downloading " + whl_file_name)

    whl_file = Path(dirname) / whl_file_name
    r = requests.get(whl_file_url, allow_redirects=True)
    whl_file.write_bytes(r.content)
    pip_install(whl_file, break_system_packages=break_system_packages)


def get_pyzed_directory() -> str:
    try:
        call_list = [sys.executable, "-m", "pip", "show", "pyzed"]
        lines = subprocess.check_output(
            call_list,
            encoding="utf-8",
            errors="replace",
        ).splitlines()

        for line in lines:
            key_word = "Location:"
            if line.startswith(key_word):
                directory = line[len(key_word) :].strip()
                if Path(directory).is_dir():
                    print("Pyzed directory is " + directory)
                    return directory + "/pyzed"
                print("ERROR : '" + directory + "' is not a directory")

        print("ERROR : Unable to find pyzed installation folder")
        print(lines)

    except Exception:
        print("ERROR : Unable to find pyzed installation folder.")

    return ""


def check_zed_sdk_version_private(file_path: str | Path) -> None:
    global ZED_SDK_MAJOR
    global ZED_SDK_MINOR

    with Path(file_path).open(encoding="utf-8") as myfile:
        data = myfile.read()

    p = re.compile("ZED_SDK_MAJOR_VERSION (.*)")
    match = p.search(data)
    if match is None:
        raise AttributeError("ZED_SDK_MAJOR_VERSION not found")
    ZED_SDK_MAJOR = match.group(1)

    p = re.compile("ZED_SDK_MINOR_VERSION (.*)")
    match = p.search(data)
    if match is None:
        raise AttributeError("ZED_SDK_MINOR_VERSION not found")
    ZED_SDK_MINOR = match.group(1)


def check_zed_sdk_version(file_path: str) -> None:
    file_path_ = file_path + "/sl/Camera.hpp"
    try:
        check_zed_sdk_version_private(file_path_)
    except AttributeError:
        file_path_ = file_path + "/sl_zed/defines.hpp"
        check_zed_sdk_version_private(file_path_)


def can_write_to_dir(dirname: str | Path) -> bool:
    path = Path(dirname)

    if not (path.exists() and path.is_dir() and os.access(path, os.W_OK)):
        return False

    try:
        testfile = path / "temp_test_file.tmp"
        with testfile.open("w") as f:
            f.write("test")
        testfile.unlink()
        return True
    except Exception:
        return False


parser = argparse.ArgumentParser(description="Helper script to download and setup the ZED Python API")
parser.add_argument("--path", help="whl file destination path")
parser.add_argument(
    "--force",
    action="store_true",
    help="Force install and use --break-system-packages (pip >= 23.0)",
)
args = parser.parse_args()

arch = platform.architecture()[0]
if arch != "64bit":
    print("ERROR : Python 64bit must be used, found " + str(arch))
    sys.exit(1)

# If path is empty, use the current working directory.
dirname = args.path or str(Path.cwd())

# If no write access, download in home.
if not can_write_to_dir(dirname):
    dirname = str(Path.home())

print("-> Downloading to '" + str(dirname) + "'")

if sys.platform == "win32":
    zed_path = os.getenv("ZED_SDK_ROOT_DIR")
    if zed_path is None:
        print("Error: you must install the ZED SDK.")
        sys.exit(1)
    else:
        check_zed_sdk_version(zed_path + "/include")

    OS_VERSION = "win" + "_" + str(ARCH_VERSION).lower()
    whl_platform_str = "win"

elif "linux" in sys.platform:
    OS_VERSION = "linux_aarch64" if "aarch64" in ARCH_VERSION else "linux_x86_64"

    zed_path = "/usr/local/zed"
    if not Path(zed_path).is_dir():
        print("Error: you must install the ZED SDK.")
        sys.exit(1)

    check_zed_sdk_version(zed_path + "/include")
    whl_platform_str = "linux"

else:
    print(f"Unknown system.platform: {sys.platform} Installation failed, see setup.py.")
    sys.exit(1)

PYTHON_MAJOR = platform.python_version().split(".")[0]
PYTHON_MINOR = platform.python_version().split(".")[1]

whl_python_version = "-cp" + str(PYTHON_MAJOR) + str(PYTHON_MINOR) + "-cp" + str(PYTHON_MAJOR) + str(PYTHON_MINOR)

if int(PYTHON_MINOR) < 8:
    whl_python_version += "m"

disp_str = "Detected platform: \n\t " + str(OS_VERSION) + "\n\t Python " + str(PYTHON_MAJOR) + "." + str(PYTHON_MINOR)
disp_str += "\n\t ZED SDK " + str(ZED_SDK_MAJOR) + "." + str(ZED_SDK_MINOR)
print(disp_str)

whl_file_name = (
    "pyzed-"
    + str(ZED_SDK_MAJOR)
    + "."
    + str(ZED_SDK_MINOR)
    + whl_python_version
    + "-"
    + whl_platform_str
    + "_"
    + str(ARCH_VERSION).lower()
    + ".whl"
)

whl_file_url = base_url + str(ZED_SDK_MAJOR) + "." + str(ZED_SDK_MINOR) + "/whl/" + OS_VERSION + "/" + whl_file_name

whl_file = Path(dirname) / whl_file_name

print("-> Checking if " + whl_file_url + " exists and is available")

try:
    r = requests.get(whl_file_url, allow_redirects=True)
    r.raise_for_status()
    whl_file.write_bytes(r.content)

except PermissionError:
    print(
        "Permission denied: Unable to write the packages. "
        "Run this script as admin or copy it in a different path and retry."
    )
    sys.exit(1)

except requests.exceptions.HTTPError as e:
    print(f"Error downloading whl file ({e})")

except requests.exceptions.RequestException as e:
    if isinstance(e, requests.exceptions.SSLError):
        print("Invalid SSL certificate, trying to fix the issue by reinstalling 'certifi' package")
        err = pip_install(
            "certifi",
            force_install=True,
            upgrade=True,
            break_system_packages=args.force,
        )

        if err == 0:
            try:
                r = requests.get(whl_file_url, allow_redirects=True)
                r.raise_for_status()
                whl_file.write_bytes(r.content)
            except Exception as e:
                print(f"Error downloading whl file ({e})")
    else:
        print(f"Error downloading whl file ({e})")


if check_valid_file(whl_file):
    print(f"-> Successfully downloaded ZED Python API wheel: {whl_file}")
    print("-> Installation will be handled by Pixi")
    sys.exit(0)
else:
    print(
        "\nUnsupported platforms, no pyzed file available for this configuration\n"
        "It can be manually installed from source "
        "https://github.com/stereolabs/zed-python-api"
    )
    sys.exit(1)
