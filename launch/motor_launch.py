import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'kit_motor_driver'
    
    return LaunchDescription([
        Node(
            package='kit_motor_driver',
            executable='motor_node',
            name='kit_motors',
        ),
    ])