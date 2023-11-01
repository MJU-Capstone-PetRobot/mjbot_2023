#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class HoldingHandNode(Node):
    def __init__(self):
        super().__init__('holding_hand_node')

        self.subscription_mode = self.create_subscription(
            String, 'mode', self.mode_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel_walk', 10)
        self.joint_states_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.should_continue = False
        self.create_timer(0.1, self.holding_hand_timer_callback)

        self.joint_efforts = {}
        self.target_cmd_vel = Twist()
        self.current_cmd_vel = Twist()

    def mode_callback(self, msg: String):
        if msg.data == "holding_hand":
            self.should_continue = True
            self.get_logger().info("Holding hand mode started!")
        else:
            self.should_continue = False

    def holding_hand_timer_callback(self):
        if self.should_continue:
            self.holding_hand()

    def joint_states_callback(self, msg: JointState):
        """Callback to handle incoming JointState messages."""
        for name, effort in zip(msg.name, msg.effort):
            self.joint_efforts[name] = effort

    def get_joint_effort(self, joint_name):
        """Returns the effort of a given joint."""
        return self.joint_efforts.get(joint_name, None)

    def holding_hand(self):

        r_shoulder_pitch_effort = self.get_joint_effort('r_shoulder_pitch')
        r_shoulder_roll_effort = self.get_joint_effort('r_shoulder_roll')
        self.get_logger().info(
            "r_shoulder_pitch_effort: {}, r_shoulder_roll_effort: {}".format(r_shoulder_pitch_effort, r_shoulder_roll_effort))

        cmd_vel = Twist()

        # Check r_shoulder_pitch effort and set linear.x
        if r_shoulder_pitch_effort is not None:
            if r_shoulder_pitch_effort > 220:
                cmd_vel.linear.x = 0.3
            elif r_shoulder_pitch_effort < -220:
                cmd_vel.linear.x = -0.3

        # Check r_shoulder_roll effort and set angular.z and modify linear.x
        if r_shoulder_roll_effort is not None:
            if r_shoulder_roll_effort > 300:
                cmd_vel.angular.z = 0.3
                cmd_vel.linear.x = 0.1 if cmd_vel.linear.x == 0 else cmd_vel.linear.x
            elif r_shoulder_roll_effort < -50:
                cmd_vel.angular.z = -0.3
                cmd_vel.linear.x = -0.1 if cmd_vel.linear.x == 0 else cmd_vel.linear.x

        self.target_cmd_vel = cmd_vel
        self.ramp_cmd_vel()

        # Publish the cmd_vel
        self.move_base(self.current_cmd_vel.linear.x,
                       self.current_cmd_vel.angular.z)

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
        cmd.linear.x = -linear_x
        cmd.angular.z = -angular_z
        self.pub_cmd_vel.publish(cmd)


if __name__ == '__main__':
    rclpy.init(args=None)

    node = HoldingHandNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
