from setuptools import find_packages, setup

package_name = "autonav_goal_selection"

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
    maintainer="Caitlyn Trievel",
    maintainer_email="ctrievel@umich.edu",
    description="Local goal selection node for autonav",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autonav_goal_selection = autonav_goal_selection.autonav_goal_selection:main",
        ],
    },
)
