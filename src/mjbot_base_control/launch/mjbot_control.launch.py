

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    controller_params_file = os.path.join(get_package_share_directory("mjbot_control"),'config','diff_drive_controller.yaml')

    # We need the robot description to be passed to the controller_manager
    # So it can check the ros2_control parameters.
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                    controller_params_file],

        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of diff_drive_controller_spawner after `joint_state_broadcaster`
    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)



# load_state_broadcaster = Node(
#             package="controller_manager",
#             executable="spawner",
#             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#             output="screen",
#         )

#     load_velocity_controller = Node(
#             package="controller_manager",
#             executable="spawner",
#             arguments=["velocity_controller", "-c", "/controller_manager"],
#             output="screen",
#         )

#     load_trajectory_controller = Node(
#             package="controller_manager",
#             executable="spawner",
#             arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
#             output="screen",
#         )
    
#     load_position_controller = Node(
#             package="controller_manager",
#             executable="spawner",
#             arguments=["forward_position_controller", "-c", "/controller_manager"],
#             output="screen",
#         )
    
#     load_statpub = Node(
#             package="robot_state_publisher",
#             executable="robot_state_publisher",
#             name="robot_state_publisher",
#             parameters=[
#                 {"robot_description": robot_description_config.toxml()}],
#             output="screen",
#         )
    
#     joint_state_broadcaster = ExecuteProcess(
#         cmd=["ros2", "control", "load_controller", "--set-state", "active", "--spin-time", "120",
#              "joint_state_broadcaster"],
#         output="screen"
#     )

#     joint_trajectory_controller = ExecuteProcess(
#         cmd=["ros2", "control", "load_controller", "--set-state", "active",
#              "joint_trajectory_controller"],
#         output="screen"
#     )
#     velocity_controller = ExecuteProcess(
#         cmd=["ros2", "control", "load_controller", "--set-state", "configure",
#              "velocity_controller"],
#         output="screen"
#     )
 