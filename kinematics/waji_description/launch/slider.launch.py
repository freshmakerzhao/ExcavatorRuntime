from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    swing = LaunchConfiguration("swing")
    boom = LaunchConfiguration("boom")
    arm = LaunchConfiguration("arm")
    bucket = LaunchConfiguration("bucket")

    return LaunchDescription(
        [
            DeclareLaunchArgument("swing", default_value="0.0"),
            DeclareLaunchArgument("boom", default_value="0.0"),
            DeclareLaunchArgument("arm", default_value="0.0"),
            DeclareLaunchArgument("bucket", default_value="0.0"),
            Node(
                package="waji_description",
                executable="waji_joint_slider.py",
                name="waji_joint_slider",
                output="screen",
                parameters=[
                    {
                        "swing_joint": swing,
                        "boom_joint": boom,
                        "arm_joint": arm,
                        "bucket_joint": bucket,
                    }
                ],
            ),
        ]
    )
