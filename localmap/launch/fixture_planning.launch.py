from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="airy_localmap",
                executable="fixture_plan_server",
                name="fixture_plan_server",
                output="screen",
            )
        ]
    )
