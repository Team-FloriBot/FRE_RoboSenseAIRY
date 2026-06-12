from glob import glob
import os

from setuptools import setup


package_name = "ground_filter_aaron"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FRE2026 Team",
    maintainer_email="fre2026@todo.todo",
    description="Python ground filter for RoboSense AIRY PointCloud2 data.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "filter_ground = ground_filter_aaron.filter_ground:main",
        ],
    },
)
