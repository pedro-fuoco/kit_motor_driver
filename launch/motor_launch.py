import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kit_motor_driver'
    motor_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'motor_params.yaml')
    return LaunchDescription([
        Node(
            package='kit_motor_driver',
            executable='motor_node',
            name='kit_motors',
            parameters=[motor_params_file],
        ),
    ])