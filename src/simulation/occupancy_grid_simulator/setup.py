from setuptools import find_packages, setup

package_name = "occupancy_grid_simulator"

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
    description="Simulates an occupancy grid from a static obstacle map for use in simulation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "occupancy_grid_simulator = occupancy_grid_simulator.occupancy_grid_simulator:main",
        ],
    },
)
