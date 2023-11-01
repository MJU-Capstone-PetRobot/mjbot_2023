#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, String
from collections import deque


class TrackingDriver(Node):
    def __init__(self):
        super().__init__('tracking_driver_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_tracker', 10)
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'owner_xyz',
            self.callback,
            10)
        self.mode_subscription = self.create_subscription(
            String,
            'mode',
            self.mode_callback,
            10)
        self.base_cmd = Twist()

        self.target_distance = 800

        self.image_width = 640
        self.image_height = 360

        # Moving average filter properties
        self.filter_length = 5
        self.data_queue = deque(maxlen=self.filter_length)
        self.current_mode = None

    def mode_callback(self, msg: String):
        # Update the current mode with the incoming message
        self.current_mode = msg.data
        if msg.data == "tracking":
            self.get_logger().info("Tracking mode started!")
        else:
            self.get_logger().warn(f"Unknown mode: {msg.data}")

    def update(self, linear_speed, angular_speed):
        self.base_cmd.linear.x = linear_speed
        self.base_cmd.angular.z = angular_speed
        self.publisher.publish(self.base_cmd)
        self.get_logger().info("Published: /cmd_vel_tracker: {}".format(self.base_cmd))

    def apply_moving_average(self, data):
        self.data_queue.append(data)
        if len(self.data_queue) < self.filter_length:
            return data
        average_data = [sum(col) / len(col) for col in zip(*self.data_queue)]
        return average_data

    def callback(self, msg):
        if self.current_mode != "tracking":
            return

        # Apply the moving average filter
        filtered_data = self.apply_moving_average(msg.data)
        if all(value == 0 for value in filtered_data):
            return
        person_x = filtered_data[0]
        person_y = filtered_data[1]
        person_distance = filtered_data[2]

        offset_x = person_x - self.image_width / 2
        theta = offset_x / self.image_width

        # angular_speed
        angular_speed = theta

        # Calculate linear_speed based on the person's distance
        if person_distance <= 800:
            linear_speed = 0.0
            angular_speed = 0.0
        elif person_distance >= 1200:
            linear_speed = 0.1  # Max speed is 0.1 when person_distance is greater than or equal to 1200
        else:
            # Linear interpolation between 0 and 0.1 based on the range of person_distance
            linear_speed = (person_distance - 800) * 0.1 / 400

        # Make sure the linear_speed doesn't exceed 0.1
        linear_speed = min(0.1, linear_speed)

        self.update(-linear_speed, -angular_speed*0.5)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
