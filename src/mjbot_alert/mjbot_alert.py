#!/usr/bin/python3
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from Alert.message_send import *


class ListeningNode(Node):
    def __init__(self):
        super().__init__('Listening_node')

        self.subscription = self.create_subscription(
            Int32, 'co_ppm', self.subscribe_callback_fire, 10)



    def subscribe_callback_fire(self, msg):

        self.get_logger().info('Received: %s' % msg.data)

        if msg.data >= 200:
            send_message(2)  # 화재 사고 발생 문자 발송
            time.sleep(1000)


def main(args=None):
    rclpy.init(args=args)
    Listening_node = ListeningNode()
    rclpy.spin(Listening_node)

    Listening_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
