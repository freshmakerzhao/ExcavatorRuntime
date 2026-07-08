import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("excavator_kinematics")
    config_path = os.path.join(package_dir, "config", "excavator_geometry.yaml")

    return LaunchDescription(
        [
            Node(
                package="excavator_kinematics",
                executable="excavator_tf_node",
                name="excavator_tf_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
