import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import random


class RandomMoveRobot(Node):

    def __init__(self):
        super().__init__('random_move_node')

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_rand', 10)
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
        if msg.data == "rand":
            self.current_mode = "rand"
            self.get_logger().info("Random move mode started!")
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
        while rclpy.ok() and self.current_mode == "rand":
            twist_msg = Twist()

            # Default behavior is to move forward
            twist_msg.linear.x = 1.0

            # If the robot is very close to an obstacle
            if self.obstacle_detected and self.last_detected_distance < 0.3:
                if not self.reversing:  # If the robot isn't already reversing
                    self.get_logger().info("Too close to an obstacle, reversing...")
                    twist_msg.linear.x = -0.5  # Move backward at half speed
                    self.reversing = True
                    self.vel_pub.publish(twist_msg)
                    rclpy.sleep(1)  # Reverse for 1 second; adjust as needed
                    self.reversing = False  # Reset the reversing flag

                # After reversing, choose a turn direction
                twist_msg.linear.x = 0  # Stop the robot

                # Determine the turn direction based on last known direction or randomly
                if self.last_turn == "right":
                    turn_direction = 1  # keep turning right
                elif self.last_turn == "left":
                    turn_direction = -1  # keep turning left
                else:
                    turn_direction = random.choice([-1, 1])
                self.last_turn = "right" if turn_direction == 1 else "left"

            # If there's an obstacle detected (but not too close)
            elif self.obstacle_detected:
                # Adapt speed based on obstacle distance
                # closer obstacles result in slower speed
                adapted_speed = min(1.0, self.safe_distance /
                                    self.last_detected_distance)
                twist_msg.linear.x = adapted_speed

                # Determine the turn direction based on where the obstacle is and previous turn
                if self.last_detected_angle > 0:  # obstacle more on the right
                    turn_direction = -1  # turn left
                elif self.last_detected_angle < 0:  # obstacle more on the left
                    turn_direction = 1  # turn right
                else:  # obstacle directly in front or no specific direction
                    if self.last_turn == "right":
                        turn_direction = 1  # keep turning right
                    elif self.last_turn == "left":
                        turn_direction = -1  # keep turning left
                    else:
                        turn_direction = random.choice([-1, 1])
                    self.last_turn = "right" if turn_direction == 1 else "left"

                twist_msg.angular.z = turn_direction * \
                    random.uniform(0.5, 1.0)  # adjust turn speed as desired

            self.vel_pub.publish(twist_msg)
            rate.sleep()


if __name__ == "__main__":
    rclpy.init()
    node = RandomMoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
