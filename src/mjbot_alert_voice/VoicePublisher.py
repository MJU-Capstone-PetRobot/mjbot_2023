# VoicePublisher.py
#! /usr/bin/env python
import rclpy
from std_msgs.msg import Int16
from rclpy.node import Node
from mjbot_alert_voice import *
from Chat.voiceChat import *
from Alert.messageSending import *

class IntPublisher(Node):

    def __init__(self):
        super().__init__('int_publisher')
        self.publisher = self.create_publisher(Int16, 'int_topic', 10)  # 'int_topic' is the topic name
        self.timer = self.create_timer(1.0, self.publish_int)  # Publish every 1 second

    def publish_int(self):
        '''
        모터 제어 / 표정
        '''
        msg = Int16()
        msg.data = voicePublish  # 1 이면 '왼손' / 2 이면 '오른손' / 11 ~ 16 까지 순서대로 기쁨 당황 분노 위험 슬픔 사랑
        self.publisher.publish(msg)
        self.get_logger().info('Published: %d' % msg.data)

class IntSubscriber(Node):

    def __init__(self):
        super().__init__('int_subscriber')
        self.subscription = self.create_subscription(Int16, 'int_topic', self.subscribe_callback, 10)

    def subscribe_callback(self, msg):
        '''
        낙상 / 배터리 잔량 / 쓰담쓰담
        '''
        self.get_logger().info('Received: %d' % msg.data)

        if msg.data == 21:  # 낙상
            speaking("할머니 괜찮으세요??")
            # send_message(1) # 낙상 사고 발생 문자 발송
        elif msg.data == 22:  # 배터리 잔량 부족
            speaking("할머니 배고파요.")
        elif msg.data == 23:  # 쓰담쓰담
            speaking("할머니 감사해요.")

def main(args=None):
    rclpy.init(args=args)
    int_publisher = IntPublisher()
    int_subscriber = IntSubscriber()
    rclpy.spin(int_publisher)
    int_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
