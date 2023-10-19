#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from std_msgs.msg import Int16
from std_msgs.msg import String  # Import String message type
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Twist

# Global variables for mode and action state
mode_selection = "default"  # Change to a string
action_state = 0  # 0: none, 1: in progress, 2: succeeded, 3: aborted, 4: rejected


class ActionClientNode(Node):
    def __init__(self):
        super().__init__('action_client_node')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self, trajectory_msg):
        global action_state
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg.trajectory
        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        action_state = 1

    def goal_response_callback(self, future):
        global action_state
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted by the action server')
        else:
            self.get_logger().info('Goal rejected by the action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        global action_state
        result = future.result().result
        self.get_logger().info('result received: {0}'.format(result))
        action_state = 2


class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        self.subscription = self.create_subscription(
            String, 'arm_mode', self.armmode_callback, 10)  # Change message type to String
        self.actionclinetnode = ActionClientNode()
        self.subscription_emotions = self.create_subscription(
            String, 'emo', self.armmove_emotion, 10)  # Change message type to String
        self.subscription_emotions  # Prevents unused variable warning
        global mode_selection
        global action_state

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_walk', 10)
        self.base_cmd = Twist()

        self.joint_names = [
            'r_shoulder_pitch',
            'r_shoulder_roll',
            'r_elbow_pitch',
            'l_shoulder_pitch',
            'l_shoulder_roll',
            'l_elbow_pitch'
        ]
        self.trajectory_msg = FollowJointTrajectory.Goal()
        self.trajectory_msg.trajectory.joint_names = self.joint_names
        self.point = JointTrajectoryPoint()

    def armmode_callback(self, msg):
        # Initialize positions with a default value
        positions = [0.0, -1.5, 0.0, 0.0, 1.5, 0.0]  # Default position
        global action_state

        # Use string values to set the positions
        if msg.data == "walk":
            mode_selection = "walk"
            self.holding_hand()
        elif msg.data == "give_right_hand":
            positions = [0.0, -1.5, 0.0, -1.5, 1.5, 0.3]
            self.get_logger().info('give right hand')
        elif msg.data == "hug":
            positions = [1.5, -1.5, -0.5, -1.5, 1.5, 0.5]
            self.get_logger().info('hugging position')

        self.point.positions = positions
        self.point.time_from_start = Duration(seconds=2).to_msg()
        self.trajectory_msg.trajectory.points = [self.point]

        # Publish the trajectory message to the action client if no other action is in progress
        if action_state == 0:
            self.actionclinetnode.send_goal(self.trajectory_msg)
            action_state = 1

    def armmove_emotion(self, msg):
        global action_state
        if msg.data == "2":  # "당황"
            positions = [0.0, 0.0, -1.5, 0.0, 0.0, -1.5]
        elif msg.data == "4":  # "분노"
            positions = [0.0, 0.0, -1.5, 0.0, 0.0, 1.5]
        # Add more emotion cases here

        self.point.positions = positions
        self.point.time_from_start = Duration(seconds=0.5).to_msg()
        self.trajectory_msg.trajectory.points = [self.point]

        # Publish the trajectory message to the action client if no other action is in progress
        if action_state == 0:
            self.actionclinetnode.send_goal(self.trajectory_msg)
            action_state = 1

        # Back to the default position after the emotion is expressed
        positions = [0.0, -1.5, 0.0, 0.0, 1.5, 0.0]  # Default position
        self.point.positions = positions
        self.point.time_from_start = Duration(seconds=0.5).to_msg()
        self.trajectory_msg.trajectory.points = [self.point]
        self.actionclinetnode.send_goal(self.trajectory_msg)

    def holding_hand(self):
        if mode_selection == "walk":
            positions = [0.0, 1.0, 0.3, 0.0, 1.5, 0.0]
            self.point.positions = positions
            self.point.time_from_start = Duration(seconds=2).to_msg()
            self.trajectory_msg.trajectory.points = [self.point]
            self.actionclinetnode.send_goal(self.trajectory_msg)
            self.sub_eff = self.create_subscription(
                DynamicJointState, '/dynamic_joint_states', self.eff_callback, 10)

    def eff_callback(self, msg):
        # Iterate over the interface values
        for interface_value in msg.interface_values:
            self.get_logger().info(f"Values: {interface_value.values[2]}")

        # Define control logic based on interface values
        if interface_value[0].values[2] > 200:
            self.base_cmd.linear.x = 0.5
        elif interface_value[0].values[2] < 200:
            self.base_cmd.linear.x = -0.5
        else:
            self.base_cmd.linear.x = 0.0

        if interface_value[1].values[2] > 200:
            self.base_cmd.angular.z = 0.5
        elif interface_value[1].values[2] < 200:
            self.base_cmd.angular.z = -0.5
        else:
            self.base_cmd.angular.z = 0.0


if __name__ == '__main__':
    rclpy.init(args=None)

    actionclinetnode = ActionClientNode()
    commander = Commander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(actionclinetnode)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
