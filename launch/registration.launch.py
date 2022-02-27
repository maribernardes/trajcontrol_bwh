import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    registration = Node(
        package = "trajcontrol",
        executable = "registration",
        parameters=[{"registration":LaunchConfiguration('registration')}]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            "registration",
            default_value="1",
            description="Registration: 0 - Use previous registration, 1 - Make new registration"
        ),
        actions.LogInfo(msg=["registration: ", LaunchConfiguration('registration')]),
        registration
    ])
