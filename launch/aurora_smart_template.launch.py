import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('trajcontrol'),
        'config',
        'aurora_smart_template_params.yaml'
        )

    sensor = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        parameters=[
            {"RIB_server_ip":"localhost"},
            {"RIB_port": "18944"},
            {"RIB_type": "client"}
        ]
    )

    robot = Node(
        package="trajcontrol",
        executable="smart_template",
        parameters=[config]
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

    user = Node(
        package="trajcontrol",
        executable="virtual_UI"
    )

    ld.add_action(sensor)
    ld.add_action(robot)
    ld.add_action(estimator)
    ld.add_action(controller)
    ld.add_action(user)

    return ld
