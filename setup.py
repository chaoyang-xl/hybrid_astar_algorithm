"""Setuptools configuration for the ROS 2 package."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "hybrid_algorithm_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="weiyu",
    maintainer_email="weiyu@users.noreply.github.com",
    description=(
        "ROS 2 Hybrid A* global planner with obstacle-aware heuristics "
        "and pure-pursuit tracking"
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            (
                "hybrid_astar_planner = "
                "hybrid_algorithm_pkg.hybrid_algorithm_planner:main"
            ),
            (
                "pure_pursuit_controller = "
                "hybrid_algorithm_pkg.pure_pursuit_controller:main"
            ),
        ],
    },
)
