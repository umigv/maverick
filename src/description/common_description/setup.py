from pathlib import Path

from setuptools import find_packages, setup

package_name = "common_description"
share = Path("share") / package_name

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[(str(share / path.parent), [str(path)]) for path in Path("urdf").rglob("*") if path.is_file()],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryan Liao",
    maintainer_email="ryanliao@umich.edu",
    description="Common xacro macros for sensors, wheels, and physics",
    license="Apache-2.0",
    extras_require={},
    entry_points={
        "console_scripts": [],
    },
)
