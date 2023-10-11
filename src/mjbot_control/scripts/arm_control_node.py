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
            String, 'arm_mode', self.mode_callback, 10)  # Change message type to String
        self.actionclinetnode = ActionClientNode()

    def mode_callback(self, msg):
        global mode_selection
        global action_state

        joint_names = [
            'r_shoulder_pitch',
            'r_shoulder_roll',
            'r_elbow_pitch',
            'l_shoulder_pitch',
            'l_shoulder_roll',
            'l_elbow_pitch'
        ]

        trajectory_msg = FollowJointTrajectory.Goal()
        trajectory_msg.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()

        # Use string values to set the positions
        if msg.data == "walk":
            positions = [0.0, 1.0, 0.3, 0.0, 1.5, 0.0]
            self.get_logger().info('walk position')
        elif msg.data == "give_right_hand":
            positions = [0.0, -1.5, 0.0, -1.5, 1.5, 0.3]
            self.get_logger().info('give right hand')
        elif msg.data == "hug":
            positions = [1.5, -1.5, -0.5, -1.5, 1.5, 0.5]
            self.get_logger().info('hugging position')
        else:
            positions = [0.0, -1.5, 0.0, 0.0, 1.5, 0.0]  # Default position

        point.positions = positions
        point.time_from_start = Duration(seconds=2).to_msg()

        trajectory_msg.trajectory.points = [point]

        # Publish the trajectory message to the action client
        if action_state == 0:
            actionclinetnode.send_goal(trajectory_msg)
            action_state = 1


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
