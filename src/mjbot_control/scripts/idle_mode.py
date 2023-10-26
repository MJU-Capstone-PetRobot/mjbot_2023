#!/usr/bin/python3
import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import time


class ActionState:
    NONE = 0
    IN_PROGRESS = 1
    SUCCEEDED = 2
    ABORTED = 3
    REJECTED = 4


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_joint_trajectory_controller/follow_joint_trajectory')
        self.action_state = ActionState.NONE
        self.action_done_event = threading.Event()

    def send_goal(self, trajectory_msg, callback=None):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg.trajectory
        self._action_client.wait_for_server()
        self.get_logger().info('Sending arm goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(callback)
        self.action_state = ActionState.IN_PROGRESS

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Arm goal accepted by the action server')
        else:
            self.get_logger().info('Arm goal rejected by the action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Arm result received: {0}'.format(result))
        self.action_state = ActionState.SUCCEEDED
        self.action_done_event.set()


class Commander(Node):
    def __init__(self, arm_controller):
        super().__init__('commander')
        self.subscription = self.create_subscription(
            String, 'arm_mode', self.arm_mode_callback, 10)
        self.subscription_emotions = self.create_subscription(
            String, 'emo', self.arm_move_emotion, 10)
        self.joint_states_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.joint_efforts = {}
        self.mode_publisher = self.create_publisher(String, 'mode', 10)
        self.mode_subscription = self.create_subscription(
            String, 'mode', self.mode_callback, 10)

        self.current_mode = "idle"

        self.position_key = 'default'

        self.new_arm_mode_received = False
        self.arm_controller = arm_controller

        self.current_cmd_vel = Twist()
        self.target_cmd_vel = Twist()

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

        self.poses = {
            'default': [0.0, -1.5, 0.0, 0.0, 1.5, 0.0],
            'give_left_hand': [0.0, -1.5, 0.0, -1.5, 1.5, 0.3],
            'give_right_hand': [1.5, -1.5, -0.3,  0.0, 1.5, 0.0],
            'hug': [1.5, -1.5, -0.5, -1.5, 1.5, 0.5],
            'emotion_2': [0.0, 0.0, -1.5, 0.0, 0.0, -1.5],
            'emotion_4': [0.0, 0.0, -1.5, 0.0, 0.0, 1.5],
            'holding_hand': [0.0, 1.0, 0.3, 0.0, 1.5, 0.0]
        }

    def mode_callback(self, msg: String):
        self.current_mode = msg.data

    def joint_states_callback(self, msg: JointState):
        """Callback to handle incoming JointState messages."""
        for name, effort in zip(msg.name, msg.effort):
            self.joint_efforts[name] = effort

    def get_joint_effort(self, joint_name):
        """Returns the effort of a given joint."""
        return self.joint_efforts.get(joint_name, None)

    def send_startup_sequence(self):
        # If the action state is NONE, send the hello_sequence
        if self.arm_controller.action_state == ActionState.NONE:
            self.set_and_send_hello_sequence()
        # Otherwise, schedule this method to be called again after a short delay
        else:
            # Check again after 0.5 seconds
            self.create_timer(0.5, self.send_startup_sequence)

    def arm_mode_callback(self, msg):
        if self.current_mode != "idle":
            return
        self.new_arm_mode_received = True
        self.position_key = 'default'  # Default position key

        if msg.data == "holding_hand":
            self.position_key = 'holding_hand'
            # Trigger HoldingHandNode

            self.current_mode = "holding_hand"
            self.set_and_send_arm_position(self.poses[self.position_key])
            self.mode_publisher.publish(String(data="holding_hand"))

            return
        elif msg.data == "give_right_hand":
            self.position_key = 'give_right_hand'
        elif msg.data == "give_left_hand":
            self.position_key = 'give_left_hand'
        elif msg.data == "hug":
            self.position_key = 'hug'
            self.set_and_send_hug_sequence()

        self.set_and_send_arm_position(self.poses[self.position_key])

    def post_action_check(self):
        # Exclude the 'walk' position from the joint effort check
        time.sleep(1)
        if self.position_key != "holding_hand":

            while True:
                if self.position_key != "default" and self.arm_controller.action_state == ActionState.SUCCEEDED:

                    r_shoulder_pitch_effort = self.get_joint_effort(
                        'r_shoulder_pitch')
                    l_shoulder_pitch_effort = self.get_joint_effort(
                        'l_shoulder_pitch')

                    # If the effort exceeds the threshold, move to default position and break out of loop
                    if r_shoulder_pitch_effort is not None and l_shoulder_pitch_effort is not None:
                        if abs(r_shoulder_pitch_effort) > 500 or abs(l_shoulder_pitch_effort) > 500:
                            self.set_and_send_arm_position(
                                self.poses['default'])
                            self.position_key = 'default'
                            self.get_logger().info('Joint effort exceeded limit. Moving to default position.')
                            break
                else:
                    break

            time.sleep(0.5)

    def arm_move_emotion(self, msg):
        self.position_key = None
        if msg.data == "2":
            self.position_key = 'emotion_2'
        elif msg.data == "4":
            self.position_key = 'emotion_4'

        if self.position_key:
            self.set_and_send_arm_position(self.poses[self.position_key])
            # Reset to default after the emotion
            self.create_timer(
                2.0, lambda: self.set_and_send_arm_position(self.poses['default']))

    def set_and_send_hug_sequence(self):
        self.trajectory_msg.trajectory.points = []
        self.add_trajectory_point(self.poses['default'], 1)
        self.add_trajectory_point([1.0, 0.0, 0.0, -1.0, 0.0, 0.0], 2)
        self.add_trajectory_point(self.poses['hug'], 5)
        self.get_logger().info('Sending hug sequence...')

        if self.arm_controller.action_state in [ActionState.NONE, ActionState.SUCCEEDED]:
            self.arm_controller.send_goal(
                self.trajectory_msg, self.arm_controller.goal_response_callback)

    def set_and_send_hello_sequence(self):
        self.trajectory_msg.trajectory.points = []
        self.add_trajectory_point(self.poses['default'], 1)
        self.add_trajectory_point([0.0, 1.0, 0.3, 0.0, 1.5, 0.0], 2)
        self.add_trajectory_point([0.0, 1.0, 0.5, 0.0, 1.5, 0.0], 2.5)
        self.add_trajectory_point([0.0, 1.0, -0.5, 0.0, 1.5, 0.0], 3)
        self.add_trajectory_point([0.0, 1.0, 0.5, 0.0, 1.5, 0.0], 3.5)
        self.add_trajectory_point([0.0, 1.0, -0.5, 0.0, 1.5, 0.0], 4)
        self.add_trajectory_point([0.0, 1.0, 0.5, 0.0, 1.5, 0.0], 4.5)

        self.add_trajectory_point(self.poses['default'], 6)
        self.get_logger().info('Sending hug sequence...')

        if self.arm_controller.action_state in [ActionState.NONE, ActionState.SUCCEEDED]:
            self.arm_controller.send_goal(
                self.trajectory_msg, self.arm_controller.goal_response_callback)

    def add_trajectory_point(self, positions, time_from_start_seconds):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            seconds=time_from_start_seconds).to_msg()
        self.trajectory_msg.trajectory.points.append(point)

    def set_and_send_arm_position(self, positions):
        self.point.positions = positions
        self.point.time_from_start = Duration(seconds=2).to_msg()
        self.trajectory_msg.trajectory.points = [self.point]
        self.get_logger().info('Sending arm goal request...')

        if self.arm_controller.action_state in [ActionState.NONE, ActionState.SUCCEEDED]:
            self.arm_controller.send_goal(
                self.trajectory_msg, self.arm_controller.goal_response_callback)

    def move_base(self, linear_x, angular_z):
        self.base_controller.move_base(linear_x, angular_z)


if __name__ == '__main__':
    rclpy.init(args=None)

    arm_controller = ArmControllerNode()
    commander = Commander(arm_controller)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(arm_controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # commander.send_startup_sequence()

    rate = commander.create_rate(50)

    try:
        while rclpy.ok():
            if arm_controller.action_done_event.is_set():
                arm_controller.action_done_event.clear()
                commander.post_action_check()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
