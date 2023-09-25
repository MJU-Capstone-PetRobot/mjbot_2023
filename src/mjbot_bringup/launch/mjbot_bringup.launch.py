#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


pkg_mjbot_bringup = get_package_share_directory('mjbot_bringup')
pkg_mjbot_control = get_package_share_directory('mjbot_control')
pkg_mjbot_description = get_package_share_directory('mjbot_description')




def generate_launch_description():

    include_mjbot_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mjbot_control, 'launch', 'mjbot_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

        # Includes mjbot_description launch file
    include_mjbot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mjbot_description, 'launch', 'mjbot_description.launch.py'),
        ),
        launch_arguments={
            'rsp': 'True',
        }.items()
    )



    
    twist_mux_params = os.path.join(pkg_mjbot_bringup,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_controller/cmd_vel_unstamped')]
        )

    mjbot_control_timer = TimerAction(period=5.0, actions=[include_mjbot_control])

    nodes = [
        include_mjbot_description,
        mjbot_control_timer,
        twist_mux
    ]

    

    return LaunchDescription(nodes

        
        )

