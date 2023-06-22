from setuptools import setup

package_name = "tutorial"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="pi@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talker = tutorial.talker:main",
            "listener = tutorial.listener:main",
            "p2p = tutorial.p2p:main",
            "all_in_one = tutorial.all_in_one:main",
        ],
    },
)
