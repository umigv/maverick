from setuptools import find_packages, setup

package_name = "path_tracking"

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
    maintainer="Anna Novak, Erika Chen",
    maintainer_email="annanova@umich.edu, erikachn@umich.edu",
    description="Path tracking for mobile robots using pure pursuit",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "path_tracking = path_tracking.path_tracking:main",
        ],
    },
)
