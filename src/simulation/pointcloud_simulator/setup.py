from setuptools import find_packages, setup

package_name = "pointcloud_simulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ethan Hardy",
    maintainer_email="102838865+hardy-ethan@users.noreply.github.com",
    description="Simulates 3D lidar and depth camera point clouds from a static obstacle map for use in simulation",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "pointcloud_simulator = pointcloud_simulator.pointcloud_simulator:main",
        ],
    },
)
