from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "marvin_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (str(Path("share") / package_name / "urdf"), glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryan Liao",
    maintainer_email="ryanliao@umich.edu",
    description="URDF description for Marvin",
    license="Apache-2.0",
    extras_require={},
    entry_points={
        "console_scripts": [],
    },
)
