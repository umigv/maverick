from setuptools import find_packages, setup

package_name = "ublox_driver"

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
    description="U-Blox ZED-F9P GPS data publisher",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ublox_driver = ublox_driver.ublox_driver:main"],
    },
)
