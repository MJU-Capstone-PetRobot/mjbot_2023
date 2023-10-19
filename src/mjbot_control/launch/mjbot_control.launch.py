import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Define robot description using ros2 param
    robot_description = Command(
        ['ros2 param get --hide-type /robot_state_publisher robot_description'])

    # Define paths to controller configuration files
    package_share_directory = get_package_share_directory("mjbot_control")
    controller_params_file = os.path.join(
        package_share_directory, 'config', 'mjbot_contoller.yaml')
    arm_control_node = Node(package='mjbot_control',
                            executable='arm_control_node.py', output='screen')
    neck_control_node = Node(package='mjbot_control',
                             executable='neck_control_node.py', output='screen')

    # Node to run the controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': ParameterValue(
                robot_description, value_type=str)},
            controller_params_file
        ],
        output="both",
    )

    # Node to spawn the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # Node to spawn the diff drive controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller",
                   "--controller-manager", "/controller_manager"],
    )

    # Node to spawn the joint trajectory controller
    load_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Delay start of diff_drive_controller_spawner after joint_state_broadcaster_spawner
    delay_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner,
                     load_trajectory_controller]
        )
    )

    # List of nodes to be launched
    nodes = [
        control_node,
        # arm_control_node,
        # neck_control_node,
        joint_state_broadcaster_spawner,
        delay_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    # Return the merged launch description
    return LaunchDescription(nodes)
