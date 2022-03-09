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
        'aurora_smart_template_params.yaml'
        )

    aurora = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        parameters=[
            {"RIB_server_ip":"localhost"},
            {"RIB_port": 18944},
            {"RIB_type": "client"}
        ]
    )

    sensor = Node(
        package = "trajcontrol",
        executable = "sensor_processing",
        parameters=[{"registration":LaunchConfiguration('registration')}]
    )

    robot = Node(
        package="trajcontrol",
        executable="smart_template"
    )

    estimator = Node(
        package="trajcontrol",
        executable="estimator_node",
        parameters=[config]
    )

    controller = Node(
        package="trajcontrol",
        executable="controller_node"
    )   

    file = Node(
        package="trajcontrol",
        executable="save_file",
        parameters=[{"filename":LaunchConfiguration('filename')}]
    )

    #user = Node(
    #    package="trajcontrol",
    #    executable="virtual_UI"
    #)

    return LaunchDescription([
        DeclareLaunchArgument(
            "filename",
            default_value="my_data",
            description="File name to save .csv file with experimental data"
        ),
        actions.LogInfo(msg=["filename: ", LaunchConfiguration('filename')]),
        aurora,
        sensor,
        estimator,
        controller,
        robot,
        file
        #user
    ])
