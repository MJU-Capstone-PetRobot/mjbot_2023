#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from Alert.messageSending import send_message, fire_check



class ListeningNode(Node):
    def __init__(self):
        super().__init__('Listening_node')
        self.subscription = self.create_subscription(Bool, 'fall_down', self.subscribe_callback, 10)


    def subscribe_callback(self, msg):

        self.get_logger().info('Received: %d' % msg.data)

        if msg.data == 1:
            send_message(1) # 낙상 사고 발생 문자 발송


def main(args=None):
    rclpy.init(args=args)
    Listening_node = ListeningNode()
    rclpy.spin(Listening_node)

    # 화재 발생 문자 전송
    fire = fire_check()
    if fire == 2:
        send_message(1)

    Listening_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
