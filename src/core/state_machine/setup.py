from setuptools import find_packages, setup

package_name = "state_machine"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryan Liao",
    maintainer_email="ryanliao@umich.edu",
    description="ROS2 state machine for publishing robot operating modes and handling state transitions",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_machine = state_machine.state_machine:main",
        ],
    },
)
