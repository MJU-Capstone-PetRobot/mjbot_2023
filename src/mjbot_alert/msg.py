#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# from Alert.messageSending import *

# 화재 감지 후, 메세지 송신
# fire = fire_check()
# if fire == 2:
#     send_message(fire)



class ListeningNode(Node):
    def __init__(self):
        super().__init__('Listening_node')
        self.subscription = self.create_subscription(Bool, 'fall_down', self.subscribe_callback, 10)


    def subscribe_callback(self, msg):

        self.get_logger().info('Received: %d' % msg.data)

        if msg.data == 1:
        send_message(1) # 낙상 사고 발생 문자 발송


def main(args=None)
    rclpy.init(args=args)
    Listening_node = ListeningNode()
    rclpy.spin(Listening_node)
    Listening_node.destroy_node()
    # 먼저 말 거는 기능

    rclpy.shutdown()


if __name__ == '__main__':
    main()

