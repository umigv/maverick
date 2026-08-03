from setuptools import setup, find_packages

package_name = "cv_self_drive"

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UMARV",
    description="CV selfdrive stack",

    entry_points={
        "console_scripts": [
            "func_tests_occ_grid = cv_self_drive.functional_tests_occ_grid:main",
        ],
    },
)