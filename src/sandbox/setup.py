import os
from glob import glob
from setuptools import find_packages, setup

package_name = "sandbox"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        exclude=["test"],
    ),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    include_package_data=True,
    maintainer_email="pi@todo.todo",
    description="My sandbox.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"planner_service = {package_name}.planner_service:main",
            f"planner_client = {package_name}.planner_client:main",
            f"bmp_launch = {package_name}.bmp_launch:generate_launch_description",
            f"cord_node = {package_name}.cord_node:main",
            f"drive = {package_name}.drive:main",
            f"kill_all = {package_name}.kill_all:reset_all",
            f"bmp = {package_name}.bmp:main",
            f"shell_input = {package_name}.shell_input:main",
            f"color = {package_name}.color:main",
            f"gyro = {package_name}.gyro:main",
            f"motor = {package_name}.motor:main",
            f"touch = {package_name}.touch:main",
            f"ultrasonic = {package_name}.ultrasonic:main",
            f"test_zed_node = {package_name}.test_zed_node:main",
        ],
    },
)
