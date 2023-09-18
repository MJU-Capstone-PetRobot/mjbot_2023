#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


import xacro

def generate_launch_description():

    robot_name = "mjbot_arm"
    package_name = robot_name + "_description"
    robot_description = os.path.join(get_package_share_directory(
    package_name), "urdf", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            package_name), "config", "controllers.yaml"
    )

    four_ws_control_node = Node(package='mjbot_arm_control', executable='mjbot_arm_control.py', output='screen')
    load_description = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        )

    load_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )

    load_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        )
    
    # load_position_controller = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["forward_position_controller", "-c", "/controller_manager"],
    #         output="screen",
    #     )
    
    load_statpub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        )
    
    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "--spin-time", "120",
             "joint_state_broadcaster"],
        output="screen"
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen"
    )
    # velocity_controller = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "configure",
    #          "velocity_controller"],
    #     output="screen"
    # )
 


    joy_node = Node(
        package = "joy",
        executable = "joy_node"
        )

    nodes = [
        joy_node,
        load_description,
        load_state_broadcaster,
  
        load_trajectory_controller,

        load_statpub,
        four_ws_control_node
    ]
    delayed_joint_trajectory_controller =RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        ),

    # delayed_joint_trajectory_controller = RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_trajectory_controller,
    #             # on_exit=[velocity_controller],
    #         )
    #     )


    

    return LaunchDescription(nodes

        
        )

