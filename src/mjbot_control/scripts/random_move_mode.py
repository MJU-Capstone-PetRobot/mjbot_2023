import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import random


class RandomMoveRobot(Node):

    def __init__(self):
        super().__init__('random_move_node')

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.range_sub = self.create_subscription(
            Range, '/range', self.range_callback, 10)

        # Subscribe to the mode topic
        self.subscription_mode = self.create_subscription(
            String, 'mode', self.mode_callback, 10)

        self.safe_distance = 0.5  # Adjust as per your robot's requirement
        self.obstacle_detected = False

        self.last_turn = None

        self.reversing = False  # A flag to check if the robot is in reverse mode

        # Add an attribute to determine the mode
        self.current_mode = "idle"

    def mode_callback(self, msg: String):
        if msg.data == "random_move":
            self.current_mode = "random_move"
            self.random_move()
        else:
            self.current_mode = "idle"
            self.stop_robot()

    def range_callback(self, data):
        self.last_detected_distance = data.range
        # Placeholder; may need adjustment.
        self.last_detected_angle = data.field_of_view

        if data.range < self.safe_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        stop_msg = Twist()
        self.vel_pub.publish(stop_msg)
        self.get_logger().info('Robot stopped.')

    def random_move(self):
        rate = self.create_rate(10)
        while rclpy.ok() and self.current_mode == "random_move":
            twist_msg = Twist()

            # ... (rest of the logic in random_move remains unchanged)

    # ... (rest of the class remains unchanged)


if __name__ == "__main__":
    rclpy.init()
    node = RandomMoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
