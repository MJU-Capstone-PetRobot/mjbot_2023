#!/usr/bin/python3
import os
from Chat.voiceChat import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
# from Alert.messageSending import *

# 화재 감지 후, 메세지 송신
# fire = fire_check()
# if fire == 2:
#     send_message(fire)

# 명자 객체 형성
mj = MYOUNGJA("순자","she")

# 대화 시작
response = mj.mic()

class TalkingNode(Node):
    def __init__(self):
        super().__init__('talking_node')
        self.publisher = self.create_publisher(Int16, 'emotions', 10)
        self.publisher = self.create_publisher(Int16, 'Arm_motions', 10)
        self.subscription = self.create_subscription(Int16, 'Dangers', self.subscribe_callback, 10)
    def publish_emotions(self, emotion):
        '''
        모터 제어 / 표정
        '''
        msg = Int16()
        msg.data = emotion
        self.publisher.publish(msg)
        self.get_logger().info('Published: %d' % msg.data)
    def publish_arm_motions(self,arm_motions):
        '''
        모터 제어 / 표정
        '''
        msg = Int16()
        msg.data = arm_motions
        self.publisher.publish(msg)
        self.get_logger().info('Published: %d' % msg.data)
    def subscribe_callback(self, msg):
        '''
        낙상 / 배터리 잔량 / 쓰담쓰담
        '''
        self.get_logger().info('Received: %d' % msg.data)

        if msg.data == 21:  # 낙상
            mj.speaking("할머니 괜찮으세요??")
            # send_message(1) # 낙상 사고 발생 문자 발송
        elif msg.data == 22:  # 배터리 잔량 부족
            mj.speaking("할머니 배고파요.")
        elif msg.data == 23:  # 쓰담쓰담
            mj.speaking("할머니 감사해요.")


def main(args=None):
    if response == "로봇":
        mj.speaker("네")
    rclpy.init(args=args)
    talking_node = TalkingNode()
    rclpy.spin(talking_node)
    while response != "":
        response = mj.mic()

        emotion = mj.gpt_send_anw[0]

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

