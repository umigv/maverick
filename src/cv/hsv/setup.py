from setuptools import find_packages, setup

package_name = "hsv"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    maintainer="UMARV",
    description="HSV library",
    entry_points={"console_scripts": []},
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
)
