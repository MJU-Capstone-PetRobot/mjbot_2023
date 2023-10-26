#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class HoldingHandNode(Node):
    def __init__(self):
        super().__init__('holding_hand_node')

        # ... Define other properties like joint_efforts, poses, etc. ...

        self.subscription_start = self.create_subscription(
            Empty, 'start_holding_hand', self.holding_hand_callback, 10)
        self.subscription_stop = self.create_subscription(
            Empty, 'stop_holding_hand', self.stop_holding_hand_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_states_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.joint_efforts = {}

        self.should_continue = False
        self.target_cmd_vel = Twist()
        self.current_cmd_vel = Twist()

    def control_callback(self, msg: String):
        if msg.data == "start":
            self.should_continue = True
            self.holding_hand()
        elif msg.data == "stop":
            self.should_continue = False
        else:
            self.get_logger().warn("Unknown command received!")

    def holding_hand_callback(self, msg):
        self.should_continue = True
        self.holding_hand()

    def stop_holding_hand_callback(self, msg):
        self.should_continue = False

    def joint_states_callback(self, msg: JointState):
        """Callback to handle incoming JointState messages."""
        for name, effort in zip(msg.name, msg.effort):
            self.joint_efforts[name] = effort

    def get_joint_effort(self, joint_name):
        """Returns the effort of a given joint."""
        return self.joint_efforts.get(joint_name, None)

    def holding_hand(self):
        while not self.new_arm_mode_received:

            r_shoulder_pitch_effort = self.get_joint_effort('r_shoulder_pitch')
            r_shoulder_roll_effort = self.get_joint_effort('r_shoulder_roll')
            self.get_logger().info('r_shoulder_pitch_effort: {0}'.format(
                r_shoulder_pitch_effort))
            self.get_logger().info('r_shoulder_roll_effort: {0}'.format(
                r_shoulder_roll_effort))

            cmd_vel = Twist()

            # ... your logic for cmd_vel based on joint efforts ...

            # Instead of immediately setting cmd_vel, set target_cmd_vel
            self.target_cmd_vel = cmd_vel
            # Call the ramp_cmd_vel method to update current_cmd_vel
            self.ramp_cmd_vel()

            # Publish the ramped cmd_vel
            self.move_base(self.current_cmd_vel.linear.x,
                           self.current_cmd_vel.angular.z)
        self.new_arm_mode_received = False

    def ramp_cmd_vel(self, alpha=0.1):
        """
        Update current_cmd_vel towards target_cmd_vel by a factor of alpha.
        """
        self.current_cmd_vel.linear.x += alpha * \
            (self.target_cmd_vel.linear.x - self.current_cmd_vel.linear.x)
        self.current_cmd_vel.angular.z += alpha * \
            (self.target_cmd_vel.angular.z - self.current_cmd_vel.angular.z)

    def move_base(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.pub_cmd_vel.publish(cmd)

    # ... other required methods like set_and_send_arm_position, get_joint_effort ...


if __name__ == '__main__':
    rclpy.init(args=None)

    node = HoldingHandNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
