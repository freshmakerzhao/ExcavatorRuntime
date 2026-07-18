from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_source",
                default_value="fixture",
                description="Fixed input provenance: fixture, replay, or live.",
            ),
            DeclareLaunchArgument(
                "execution_mode",
                default_value="shadow",
                description="Fixed execution mode; this package accepts shadow only.",
            ),
            Node(
                package="airy_mission_runtime",
                executable="machine_behavior_shadow_server",
                name="machine_behavior_shadow_server",
                output="screen",
                parameters=[
                    {
                        "input_source": LaunchConfiguration("input_source"),
                        "execution_mode": LaunchConfiguration("execution_mode"),
                    }
                ],
            ),
        ]
    )
