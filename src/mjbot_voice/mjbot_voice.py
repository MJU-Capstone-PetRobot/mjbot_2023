#!/usr/bin/python3
import os
from Chat.voiceChat import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32, Bool
import threading

"""
Publisher : 감정(std_msgs.msg/String) , 팔 제어(std_msgs.msg/String)
Subscriber : 낙상(std_msgs.msg/Bool) , 잔량(std_msgs.msg/String) , 쓰담쓰담(std_msgs.msg/Bool) , 일산화탄소(exmaple_interfaces.msg/Int32)  
"""


class TalkingNode(Node):
    def __init__(self):
        super().__init__('talking_node')
        self.get_logger().info("Talking Node")
        self.publisher_emotions = self.create_publisher(
            String, 'emo', 10)  # Updated topic name and message type
        self.publisher_arm_mode = self.create_publisher(String, 'arm_mode', 10)

        # self.publisher_walk = self.create_publisher(Int16, 'walk', 10)

    # def publish_walk(self, walk):
    #     '''
    #     산책
    #     '''
    #     msg = Int16()
    #     msg.data = walk
    #     self.publisher_walk.publish(msg)
    #     self.get_logger().info('Published: %d' % msg.data)

    def publish_emotions(self, emotion):
        '''
        표정
        '''
        msg = String()  # Updated message type to String
        msg.data = str(emotion)
        self.publisher_emotions.publish(msg)
        self.get_logger().info('Published: %s' % msg.data)  # Updated log message format

    def publish_arm_motions(self, Arm_motions):
        '''
        모터 제어
        '''
        msg = String()
        msg.data = Arm_motions
        self.publisher_arm_mode.publish(msg)
        self.get_logger().info('Published: %s' % msg.data)


class VoiceSuscriber(Node):
    def __init__(self):
        super().__init__('hear_node')
        self.subscription = self.create_subscription(
            Bool, 'fall_down', self.subscribe_callback_fall_down, 10)
        self.subscription = self.create_subscription(
            String, 'Bat_state', self.subscribe_callback_bat_state, 10)
        self.subscription = self.create_subscription(
            Bool, 'touch', self.subscribe_callback_touch, 10)
        self.subscription = self.create_subscription(
            Int32, 'co', self.subscribe_callback_co, 10)

    def subscribe_callback_fall_down(self, msg):
        '''
        낙상
        '''
        self.get_logger().info(f'Received: {msg.dat}')

        if msg.data == True:
            speaking("할머니 괜찮으세요??")

    def subscribe_callback_bat_state(self, msg):
        '''
        화재
        '''
        self.get_logger().info('Received: %s' % msg.data)

        if msg.data <= 45:
            speaking("할머니 배고파요")

    def subscribe_callback_touch(self, msg):
        '''
        터치
        '''
        self.get_logger().info(f'Received: {msg.data}')

        if msg.data == True:
            speaking("깔깔깔")

    def subscribe_callback_co(self, msg):
        '''
        화재
        '''
        self.get_logger().info('Received: %d' % msg.data)

        if msg.data >= 200:
            speaking("할머니 불이 났어요!!")


def main(args=None):
    global common
    common = 0
    rclpy.init(args=args)
    talking_node = TalkingNode()
    hear_node = VoiceSuscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(talking_node)
    executor.add_node(hear_node)

    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    while 1:
        # 먼저 말 거는 기능 실험용
        if common == 0:
            # speak_first_ex()

            common = 1
        # 먼저 말 거는 기능
        # speak_first()
        # 대화 시작
        response = mic()
        if response == "로봇":
            speaking("네")

            response = mic()
            if response == "초기화":
                speaking("초기화를 진행합니다!")
                name_ini()

            check = name_check()
            name_check_ = check[0]
            value_check = check[1]

            mj = MYOUNGJA(name_check_, value_check)
            while response != "":
                # talking_node.publish_emotions("6")
                response_ = mj.gpt_send_anw(response)
                emotion = response_[0]

                # NULL, close, moving, wink, angry, sad, daily
                if emotion == "평범":
                    talking_node.publish_emotions("6")
                elif emotion == "당황":
                    talking_node.publish_emotions("2")
                elif emotion == "분노":
                    talking_node.publish_emotions("4")
                elif emotion == "슬픔":
                    talking_node.publish_emotions("5")
                elif response == "산책 가자":  # 산책 가자
                    talking_node.publish_arm_motions("walk")
                elif response == "오른손":  # 오른손
                    talking_node.publish_arm_motions("give_right_hand")
                elif response == "왼손":  # 왼손
                    talking_node.publish_arm_motions("give_left_hand")
                elif response == "안기":  # 안기
                    talking_node.publish_arm_motions("hug")
                else:
                    talking_node.publish_emotions("0")

                os.remove("sampleWav.wav")

                ans = response_[1]

                speaking(ans)
                talking_node.publish_emotions("6")

                response = mic()

                if response == "":
                    os.remove("sampleWav.wav")
                    break

    executor_thread.join()
    talking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
