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
        self.publisher_mode = self.create_publisher(String, 'mode', 10)

    def publish_mode(self, mode):
        '''
        모드 제어
        '''
        msg = String()
        msg.data = str(mode)
        self.publisher_mode.publish(msg)
        self.get_logger().info('Published: %s' % msg.data)

    def publish_arm_motions(self, Arm_motions):
        '''
        모터 제어
        '''
        msg = String()
        msg.data = str(Arm_motions)
        self.publisher_arm_mode.publish(msg)
        self.get_logger().info('Published: %s' % msg.data)

    def publish_emotions(self, emotions):
        '''
        감정 제어
        '''
        msg = String()
        msg.data = str(emotions)
        self.publisher_emotions.publish(msg)
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
    import os
    from os import path
    global common
    import time
    common = 0
    rclpy.init(args=args)
    talking_node = TalkingNode()
    hear_node = VoiceSuscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(talking_node)
    executor.add_node(hear_node)

    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    if path.exists("./sampleWav.wav"):
        os.remove("./sampleWav.wav")
    if path.exists("./ResultMP3.mp3"):
        os.remove("./ResultMP3.mp3")
    if path.exists("yes.wav"):
        os.remove("./yes.wav")

    call_num = 0
    while 1:
        name_check()

        # modes : tracking, holding_hand, idle, random_move
        mj = MYOUNGJA()

        # 먼저 말 거는 기능 실험용
        if common == 0:
            use_sound("./mp3/ex1.wav")
            time.sleep(2)
            use_sound("./mp3/ex_2.wav")
        common = 1
        # 먼저 말 거는 기능
        # speak_first()

        # 대화 시작
        response = mic(2)
        if response == "":
            call_num += 1
            print(call_num)
        if call_num == 3:
            use_sound("./mp3/say_my_name.wav")
            call_num = 0
        if response == "로봇":
            use_sound("./mp3/yes.wav")
            # 대답 기다리는 동안 표정 변화
            talking_node.publish_emotions("mic_waiting")
            response = mic(3)
            talking_node.publish_emotions("daily")

            if response == "초기화":
                use_sound("./mp3/reset.wav")
                name_ini()
            elif response == "종료":
                use_sound("./mp3/off.wav")
                break
            elif response == "조용":
                use_sound("./mp3/quiet.wav")
                call_num = - 1000000

            while response != "":
                if response == "산책 가자":  # 산책 가자
                    talking_node.publish_arm_motions("walk")
                    talking_node.publish_mode("tracking")
                elif response == "손잡자":  # 손잡자
                    talking_node.publish_mode("holding_hand")
                elif response == "손놔":  # 손놔
                    talking_node.publish_arm_motions("idle")
                elif response == "오른손":  # 오른손
                    talking_node.publish_arm_motions("give_right_hand")
                elif response == "왼손":  # 왼손
                    talking_node.publish_arm_motions("give_left_hand")
                elif response == "안기":  # 안기
                    talking_node.publish_arm_motions("hug")
                elif response == "조용":
                    break
                else:
                    response_ = mj.gpt_send_anw(response)
                    emotion = response_[0]

                    # close, moving, wink, angry, sad, daily
                    if emotion == "평범":
                        talking_node.publish_emotions("daily")
                    elif emotion == "당황":
                        talking_node.publish_emotions("panic")
                    elif emotion == "분노":
                        talking_node.publish_emotions("angry")
                    elif emotion == "슬픔":
                        talking_node.publish_emotions("sad")
                    else:
                        talking_node.publish_emotions("daily")
                    ans = response_[1]

                    speaking(ans)
                    talking_node.publish_emotions("daily")
                os.remove("sampleWav.wav")

                talking_node.publish_emotions("mic_wating")
                response = mic(3)

                if response == "":
                    os.remove("sampleWav.wav")
                    break

    executor_thread.join()
    talking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
