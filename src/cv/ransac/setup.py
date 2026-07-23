from setuptools import find_packages, setup

package_name = "ransac"

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
    maintainer="the2nake",
    maintainer_email="41810559+the2nake@users.noreply.github.com",
    description="Provides a pipeline to extract the driveable area from RGB-D images given by the ZED camera",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
)
