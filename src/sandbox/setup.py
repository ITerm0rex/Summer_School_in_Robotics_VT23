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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="pi@todo.todo",
    description="My sandbox.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"kill_all = {package_name}.kill_all:reset_all",
            f"my_motor = {package_name}.my_motor:main",
            f"bmp = {package_name}.bmp:main",
            f"shell_input = {package_name}.shell_input:main",
            f"all_in_one = {package_name}.all_in_one:main",
            f"talker = {package_name}.talker:main",
            f"color = {package_name}.color:main",
            f"gyro = {package_name}.gyro:main",
            f"motor_v2 = {package_name}.motor_v2:main",
            f"motor = {package_name}.motor:main",
            f"touch = {package_name}.touch:main",
            f"ultrasonic = {package_name}.ultrasonic:main",
        ],
    },
)
