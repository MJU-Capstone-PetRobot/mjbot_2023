#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
    def joy_callback(self, msg):
        # Print the joystick axes and buttons
        self.get_logger().info("Axes: %s", msg.axes)
        self.get_logger().info("Buttons: %s", msg.buttons)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
       #controller,