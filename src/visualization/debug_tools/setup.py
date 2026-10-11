from setuptools import find_packages, setup

package_name = "debug_tools"

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
    maintainer="tutu",
    maintainer_email="tranbao@umich.edu",
    description="Robot debugging and data visualization tools",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "path_history = debug_tools.path_history:main",
        ],
    },
)
