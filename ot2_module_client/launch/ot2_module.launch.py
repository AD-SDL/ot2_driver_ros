
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

from os import getenv

def generate_launch_description():
    ld = LaunchDescription()

    ip = LaunchConfiguration('ip')
    robot_name = LaunchConfiguration('robot_name')

    declare_use_ip_cmd = DeclareLaunchArgument(
        name='ip',
        default_value="146.137.240.35",
        description='Flag to accept ip address')

    declare_use_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value="OT2_alpha",
        description='Flag to accept robot_name')

    ot2 = Node(
            package='ot2_module_client',
            executable='ot2Node',
            namespace = 'std_ns',
            name=robot_name,
            output='screen',
            prameters = [
                {"ip":ip}
                ],
            emulate_tty = True,
        )

    ld.add_action(declare_use_ip_cmd)
    ld.add_action(declare_use_robot_name_cmd)
    ld.add_action(ot2)
    return ld
