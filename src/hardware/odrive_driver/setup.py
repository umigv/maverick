from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "odrive_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (str(Path("share") / package_name / "sounds"), glob("sounds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ericb",
    maintainer_email="ericb@umich.edu",
    description="Dual ODrive motor controller driver",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "odrive_driver = odrive_driver.odrive_driver:main",
        ],
    },
)
