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

mode_selection = 0  # 1: opposite phase, 2: in-phase, 3: pivot turn, 4: none
action_state = 0 # 0: none, 1: in progress, 2: succeeded, 3: aborted, 4: rejected

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('action_client_node')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        

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
        # This callback will be called when the goal response (accepted or rejected) is received.
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted by the action server')
        else:
            self.get_logger().info('Goal rejected by the action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        global action_state
        # This callback will be called when the result response is received.
        result = future.result().result
        self.get_logger().info('result received: {0}'.format(result))
        action_state = 2

        
class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.actionclinetnode = ActionClientNode()
        
        

    def timer_callback(self):
        global mode_selection
        global action_state
        #self.get_logger().info('timer_callback')

        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Adjust joint names as needed

        trajectory_msg = FollowJointTrajectory.Goal()
        trajectory_msg.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()

        if mode_selection == 1:  # opposite phase
            positions = [0.0, 1.0, 0.3, 0.0, 1.5, 0.0]
            self.get_logger().info('opposite phase')
        elif mode_selection == 2:  # in-phase
            positions = [0.0, -1.5, 0.0, -1.5, 1.5, 0.3]
            self.get_logger().info('in-phase')
        elif mode_selection == 3:  # pivot turn
            positions = [1.5, -1.5, -0.5, -1.5, 1.5, 0.5]
            self.get_logger().info('pivot turn')
        else:
            positions = [0.0, -1.5, 0.0, 0.0, 1.5, 0.0]

        point.positions = positions
        point.time_from_start = Duration(seconds=2).to_msg()

        trajectory_msg.trajectory.points = [point]

        
        return trajectory_msg

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global mode_selection

        if data.buttons[0] == 1:  # in-phase # A button of Xbox 360 controller
            mode_selection = 2
            self.get_logger().info('in-phase')
        elif data.buttons[4] == 1:  # opposite phase # LB button of Xbox 360 controller
            mode_selection = 1
        elif data.buttons[5] == 1:  # pivot turn # RB button of Xbox 360 controller
            mode_selection = 3
        else:
            mode_selection = 4

if __name__ == '__main__':
    rclpy.init(args=None)

    actionclinetnode = ActionClientNode()
    commander = Commander()
    joy_subscriber = JoySubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)
    executor.add_node(actionclinetnode)
    trajectory_msg = commander.timer_callback()
    if action_state == 0:
        actionclinetnode.send_goal(trajectory_msg)
        action_state = 1



    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)

    while True:
        if action_state == 2:
            trajectory_msg = commander.timer_callback()
            action_state = 0
            actionclinetnode.send_goal(trajectory_msg)
        

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()
