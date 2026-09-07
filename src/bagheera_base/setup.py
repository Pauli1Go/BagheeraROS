from glob import glob
from setuptools import find_packages, setup


package_name = "bagheera_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Paul Praschl",
    maintainer_email="Pauli1Go@users.noreply.github.com",
    description="ROS 2 host driver and manual control for the Bagheera mobile base.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bagheera_base_driver = bagheera_base.base_driver:main",
            "bagheera_controller = bagheera_base.controller_teleop:main",
            "bagheera_manual_mode = bagheera_base.manual_mode:main",
        ],
    },
)
