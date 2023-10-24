import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Define paths and configurations
    robot_description = Command(
        ['ros2 param get --hide-type /robot_state_publisher robot_description'])
    package_share_directory = get_package_share_directory("mjbot_control")
    controller_params_file = os.path.join(
        package_share_directory, 'config', 'mjbot_controller.yaml')

    # Define nodes
    control_node = create_control_node(
        robot_description, controller_params_file)
    joint_state_broadcaster_spawner = create_spawner_node(
        "joint_state_broadcaster")
    diff_drive_controller_spawner = create_spawner_node("diff_controller")
    load_trajectory_controller = create_spawner_node(
        "arm_joint_trajectory_controller")
    arm_control_node = Node(package='mjbot_control',
                            executable='arm_control_node.py', output='screen')
    neck_control_node = Node(package='mjbot_control',
                             executable='neck_control_node.py', output='screen')

    # Define event handlers for delayed starts
    delay_diff_drive_after_joint_state = create_delay_handler(
        joint_state_broadcaster_spawner, diff_drive_controller_spawner)
    delay_trajectory_after_diff_drive = create_delay_handler(
        diff_drive_controller_spawner, load_trajectory_controller)
    delay_arm_after_joint_state = create_delay_handler(
        joint_state_broadcaster_spawner, arm_control_node)

    # Return the merged launch description
    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_after_joint_state,
        delay_trajectory_after_diff_drive,
        delay_arm_after_joint_state,
        # neck_control_node,
    ])


def create_control_node(robot_description, controller_params_file):
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': ParameterValue(
                robot_description, value_type=str)},
            controller_params_file
        ],
        output="both",
    )


def create_spawner_node(controller_name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name,
                   "--controller-manager", "/controller_manager"],
    )


def create_delay_handler(target_action, on_exit_action):
    return RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=target_action,
            on_exit=[on_exit_action]
        )
    )
