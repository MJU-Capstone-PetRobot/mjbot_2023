#!/usr/bin/python3
import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

# Global variables for action state
action_state = 0  # 0: none, 1: in progress, 2: succeeded, 3: aborted, 4: rejected

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_joint_trajectory_controller/joint_trajectory')

    def send_goal(self, trajectory_msg):
        global action_state
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg.trajectory
        self._action_client.wait_for_server()
        self.get_logger().info('Sending arm goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        action_state = 1

    def goal_response_callback(self, future):
        global action_state
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Arm goal accepted by the action server')
        else:
            self.get_logger().info('Arm goal rejected by the action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        global action_state
        result = future.result().result
        self.get_logger().info('Arm result received: {0}'.format(result))
        action_state = 2

class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('base_controller_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_walk', 10)
        self.base_cmd = Twist()

    def move_base(self, linear_x, angular_z):
        self.base_cmd.linear.x = linear_x
        self.base_cmd.angular.z = angular_z
        self.publisher_.publish(self.base_cmd)

class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        self.subscription = self.create_subscription(
            String, 'arm_mode', self.arm_mode_callback, 10)
        self.subscription_emotions = self.create_subscription(
            String, 'emo', self.arm_move_emotion, 10)
        self.subscription_emotions
        global action_state

        self.arm_controller = ArmControllerNode()
        self.base_controller = BaseControllerNode()

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

    def arm_mode_callback(self, msg):
        positions = [0.0, -1.5, 0.0, 0.0, 1.5, 0.0]  # Default position
        global action_state

        if msg.data == "walk":
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
        self.get_logger().info('Sending arm goal request...')
        ##show action state
        self.get_logger().info('action state: {0}'.format(action_state))

        if action_state == 0:
            self.arm_controller.send_goal(self.trajectory_msg)
            
            action_state = 1

    def arm_move_emotion(self, msg):
        global action_state
        if msg.data == "2":  # "당황"
            positions = [0.0, 0.0, -1.5, 0.0, 0.0, -1.5]
        elif msg.data == "4":  # "분노"
            positions = [0.0, 0.0, -1.5, 0.0, 0.0, 1.5]

        self.point.positions = positions
        self.point.time_from_start = Duration(seconds=0.5).to_msg()
        self.trajectory_msg.trajectory.points = [self.point]

        if action_state == 0:
            self.arm_controller.send_goal(self.trajectory_msg)
            action_state = 1

        positions = [0.0, -1.5, 0.0, 0.0, 1.5, 0.0]  # Default position
        self.point.positions = positions
        self.point.time_from_start = Duration(seconds=0.5).to_msg()
        self.trajectory_msg.trajectory.points = [self.point]
        self.arm_controller.send_goal(self.trajectory_msg)

    def holding_hand(self):
        if action_state == 0:
            positions = [0.0, 1.0, 0.3, 0.0, 1.5, 0.0]
            self.point.positions = positions
            self.point.time_from_start = Duration(seconds=2).to_msg()
            self.trajectory_msg.trajectory.points = [self.point]
            self.arm_controller.send_goal(self.trajectory_msg)

    def move_base(self, linear_x, angular_z):
        self.base_controller.move_base(linear_x, angular_z)

if __name__ == '__main__':
    rclpy.init(args=None)

    actionclinetnode = ArmControllerNode()
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
