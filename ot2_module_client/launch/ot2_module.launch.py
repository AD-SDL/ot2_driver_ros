
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

from os import getenv

def generate_launch_description():
    ld = LaunchDescription()

    ot2 = Node(
            package='ot2_module_client',
            executable='ot2Node',
            namespace = 'std_ns',
            name=getenv('robot_name'),
            output='screen',
            emulate_tty = True,
        )

    ld.add_action(ot2)
    return ld
