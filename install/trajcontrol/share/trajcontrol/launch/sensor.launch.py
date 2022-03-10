from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sensor = Node(
        package = "trajcontrol",
        executable = "sensor_processing",
        parameters=[{"registration":LaunchConfiguration('registration')}]
    )

    ld.add_action(sensor)
    return ld