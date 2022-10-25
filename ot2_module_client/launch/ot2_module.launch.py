
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ot2 = Node(
            package='ot2_module_client',
            executable='ot2Node',
            name='ot2Node',
            output='screen',
            emulate_tty = True,
        )

    ld.add_action(ot2)
    return ld