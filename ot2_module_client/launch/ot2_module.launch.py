from os import getenv
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ot2_module_client',
            namespace=getenv('robot_name', 'ot2_module'),
            executable='ot2Node',
            name='ot2Node',
            output='screen',
            emulate_tty = True,
        ),
    ])