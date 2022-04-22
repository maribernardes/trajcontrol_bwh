import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('trajcontrol'),
        'config',
        'virtual_nodes_params.yaml'
        )

    aurora = Node(
        package="trajcontrol",
        executable="virtual_aurora",
    )

    sensor = Node(
        package = "trajcontrol",
        executable = "sensor_processing",
        parameters=[{"registration":LaunchConfiguration('registration')}]
    )

    robot = Node(
        package="trajcontrol",
        executable="virtual_robot"
    )

    estimator = Node(
        package="trajcontrol",
        executable="estimator_node",
        parameters=[config]
    )

    controller = Node(
        package="trajcontrol",
        executable="robot_cmd"
        # executable="controller_node"
    )   

    file = Node(
        package="trajcontrol",
        executable="save_file",
        parameters=[{"filename":LaunchConfiguration('filename')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "filename",
            default_value="my_data",
            description="File name to save .csv file with experimental data"
        ),
        actions.LogInfo(msg=["filename: ", LaunchConfiguration('filename')]),
        DeclareLaunchArgument(
            "registration",
            default_value="0",
            description="0=load previous / 1=new registration"
        ),
        actions.LogInfo(msg=["registration: ", LaunchConfiguration('registration')]),
        aurora,
        sensor,
        estimator,
        controller,
        robot,
        file
    ])
