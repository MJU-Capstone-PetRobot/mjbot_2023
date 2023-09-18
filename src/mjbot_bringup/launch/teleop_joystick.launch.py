
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    joystick_config = os.path.join(get_package_share_directory('mjbot_bringup'),'config','joystick.yaml')

    cmd_vel_topic_arg = DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel_joy',
            description='Indicates the cmd_vel topic.')
    cmd_vel_topic =  LaunchConfiguration('cmd_vel_topic')

    joy_linux_node = Node(
            package='joy_linux',
            executable='joy_linux_node',
            parameters=[joystick_config],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joystick_config],
            remappings=[('/cmd_vel', cmd_vel_topic)]
         )

    return LaunchDescription([
        cmd_vel_topic_arg,
        joy_linux_node,
        teleop_node,
    ])