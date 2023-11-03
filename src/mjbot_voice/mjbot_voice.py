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

        import json

        write_data = {
            "bat_state" : msg.data
        }
        with open('./bat_value.json', 'w') as d:
            json.dump(write_data, d)

        if msg.data <= 40:
            speaking("할머니 배고파요")


    # def subscribe_callback_touch(self, msg):
    #     '''
    #     터치
    #     '''
    #     self.get_logger().info(f'Received: {msg.data}')

    #     if msg.data == True:
    #         speaking("깔깔깔")

    def subscribe_callback_co(self, msg):
        '''
        화재
        '''
        self.get_logger().info('Received: %d' % msg.data)

        if msg.data >= 200:
            speaking("할머니 불이 났어요!!")


def main(args=None):
    import json
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

        # 배터리 잔량 체크
        with open('./bat_value.json', 'r') as f:
            data = json.load(f)
            bat_state = data["bat_state"]

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
            elif response == "배터리":
                speaking(f"배터리 잔량은 {bat_state} 퍼센트 입니다.")


            while response != "":
                if response == "산책 가자":  # 산책 가자
                    talking_node.publish_arm_motions("holding_hand")
                    talking_node.publish_mode("holding_hand")
                elif response == "따라와":  # 따라와
                    talking_node.publish_mode("tracking")
                elif response == "멈춰":  # 멈춰
                    talking_node.publish_mode("idle")
                elif response == "오른손":  # 오른손
                    talking_node.publish_arm_motions("give_right_hand")
                elif response == "왼손":  # 왼손
                    talking_node.publish_arm_motions("give_left_hand")
                elif response == "안아줘":  # 안기
                    talking_node.publish_arm_motions("hug")
                elif response == "조용":
                    break
                else:
                    response_ = mj.gpt_send_anw(response)
                    emotion = response_[0]

                    # close, moving, wink, angry, sad, daily
                    if emotion == "daily":
                        talking_node.publish_emotions("daily")
                    elif emotion == "surprise":
                        talking_node.publish_emotions("moving")
                    elif emotion == "angry":
                        talking_node.publish_emotions("angry")
                    elif emotion == "sad":
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
