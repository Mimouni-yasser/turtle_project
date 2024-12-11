from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="capture_images",
            executable="capture_images",
            output="screen",
        ),
        Node(
            package="capture_images",
            executable="robot_project",
            output="screen"
        )
    ])