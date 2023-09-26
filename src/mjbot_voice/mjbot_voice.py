#!/usr/bin/python3
import os
from Chat.voiceChat import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16 , Int32, Bool, String

"""
Publisher : 산책(Int16) , 감정(Int16) , 모터 제어(Int16)
Subscriber : 위험 감지(Int16) , 낙상(Bool) , 잔량(String) , 쓰담쓰담(Bool) , 일산화탄소(Int32)  
"""

class TalkingNode(Node):
    def __init__(self):
        super().__init__('talking_node')
        self.get_logger().info("Talking Node")
        self.publisher = self.create_publisher(Int16, 'emotions', 10)
        self.publisher = self.create_publisher(Int16, 'Arm_motions', 10)
        self.publisher = self.create_publisher(Int16, 'walk', 10)


    def publish_walk(self,walk):
        '''
        모터 제어
        '''
        msg = Int16()
        msg.data = walk
        self.publisher.publish(msg)
        self.get_logger().info('Published: %d' % msg.data)

    def publish_emotions(self, emotion):
        '''
        표정
        '''
        msg = Int16()
        msg.data = emotion
        self.publisher.publish(msg)
        self.get_logger().info('Published: %d' % msg.data)

    def publish_arm_motions(self,Arm_motions):
        '''
        모터 제어
        '''
        msg = Int16()
        msg.data = Arm_motions
        self.publisher.publish(msg)
        self.get_logger().info('Published: %d' % msg.data)

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

class VoiceSuscriber(Node):
    def __init__(self):
        super().__init__('Sub_node')
        self.subscription = self.create_subscription(Int16, 'Dangers', self.subscribe_callback, 10)
        self.subscription = self.create_subscription(Bool, 'fall_down', self.subscribe_callback, 10)
        self.subscription = self.create_subscription(String, 'Bat_state', self.subscribe_callback, 10)
        self.subscription = self.create_subscription(Bool, 'touch', self.subscribe_callback, 10)
        self.subscription = self.create_subscription(Int32, 'co', self.subscribe_callback, 10)

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
    talking_node = TalkingNode()
    hear_node = VoiceSuscriber()

    rclpy.spin(talking_node)
    rclpy.spin(hear_node)
    # 명자 객체 형성
    mj = MYOUNGJA("순자", "she")

    # 대화 시작
    response = mj.mic()
    if response == "로봇":
        mj.speaker("네")

        while response != "":
            response = mj.mic()

            emotion = mj.gpt_send_anw(response)[0]

            # 기쁨, 슬픔, 평범, 당황, 분노, 사랑, 위험
            if emotion == "기쁨":
                talking_node.publish_emotions(11)
                #voicePublish = 11

            elif emotion == "당황":
            
                #voicePublish = 12
                talking_node.publish_emotions(12)
            elif emotion == "분노":
                #voicePublish = 13
                talking_node.publish_emotions(13)
            elif emotion == "위험":
                #voicePublish = 14
                talking_node.publish_emotions(14)
            elif emotion == "슬픔":
                #voicePublish = 15
                talking_node.publish_emotions(15)
            elif emotion == "사랑":
                #voicePublish = 16
                talking_node.publish_emotions(16)
            elif emotion == "평범" and response == "왼손": # 왼손
                #voicePublish = 1
                talking_node.publish_arm_motions(1)
            elif emotion == "평범" and response == "오른손": # 오른손
                #voicePublish = 2
                talking_node.publish_arm_motions(2)
            else:
                #voicePublish = 0
                talking_node.publish_emotions(0)

            os.remove("sampleWav.wav")

            ans = mj.gpt_send_anw(response)[1]

            mj.speaker(ans)

    # 먼저 말 거는 기능
    mj.speak_first()
    talking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
