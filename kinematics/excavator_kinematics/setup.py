from glob import glob
from setuptools import find_packages, setup

package_name = "excavator_kinematics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Excavator TF tree publisher and bucket tip forward kinematics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "excavator_tf_node = excavator_kinematics.excavator_tf_node:main",
            "joint_slider_publisher = excavator_kinematics.joint_slider_publisher:main",
        ],
    },
)
