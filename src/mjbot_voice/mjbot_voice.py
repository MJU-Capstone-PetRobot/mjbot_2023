#!/usr/bin/env python3
import json
import os
import threading
import time
import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from openwakeword.model import Model
from Chat.voiceChat import *

# File Paths
SAMPLE_WAV = "./sampleWav.wav"
RESULT_MP3 = "./ResultMP3.mp3"
YES_WAV = "./yes.wav"
LANGUAGE = "user_data/language_select.json"

# Audio Configuration
CHANNELS = 1
RATE = 16000
CHUNK = 120


def file_cleanup():
    for file_path in [SAMPLE_WAV, RESULT_MP3, YES_WAV]:
        remove_file_if_exists(file_path)


# Helper function to parse the battery status

def remove_file_if_exists(file_path):
    if os.path.exists(file_path):
        os.remove(file_path)


def start_executor_thread(executor):
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    return executor_thread


# Talking Node definition


class TalkingNode(Node):
    def __init__(self):
        super().__init__('talking_node')
        self.get_logger().info("Talking Node initialized")
        self.publisher_emotions = self.create_publisher(String, 'emo', 10)
        self.publisher_arm_mode = self.create_publisher(String, 'arm_mode', 10)
        self.publisher_mode = self.create_publisher(String, 'mode', 10)

    def publish_mode(self, mode):
        '''
        mode
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
        self._publish_message(emotions, self.publisher_emotions)

    def _publish_message(self, message, publisher):
        msg = String()
        msg.data = str(message)
        publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


# Voice Subscriber Node definition


class VoiceSubscriber(Node):
    def __init__(self):
        super().__init__('hear_node')
        self.get_logger().info("hear Node initialized")
        self.subscription = self.create_subscription(
            Bool, 'owner_fall', self.subscribe_callback_fall_down, 10)
        # self.subscription = self.create_subscription(
        #     Bool, 'touch', self.subscribe_callback_touch, 10)
        self.subscription = self.create_subscription(
            Int32, 'co_ppm', self.subscribe_callback_co, 10)
        self.subscription = self.create_subscription(
            String, 'bat_percent', self.subscribe_callback_bat_state, 10)
        self.subscription = self.create_subscription(
            String, 'bat_time', self.subscribe_callback_bat_time, 10)

    def subscribe_callback_fall_down(self, msg):
        '''
        낙상
        '''
        self.get_logger().info(f'Received: {msg.data}')
        lang = language_check()
        if msg.data == True and lang == 1:
            speaking("괜찮으세요??")
            time.sleep(20)
        elif msg.data == True and lang == 0:
            speaking_en("Are you ok??")


    def subscribe_callback_bat_state(self, msg):
        import json
        self.get_logger().info('Received: %s' % msg.data)

        bat_state = float(msg.data)
        write_data = {
            "bat_state": bat_state,
        }

        with open('user_data/bat_percent.json', 'w') as d:
            json.dump(write_data, d)

        lang = language_check()
        if bat_state <= 40.0 and lang == 1:
            speaking("배고파요")
        elif bat_state <= 40.0 and lang == 0:
            speaking_en("I'm hungry")

    def subscribe_callback_bat_time(self, msg):
        import json
        self.get_logger().info('Received: %s' % msg.data)

        try:
            hours, minutes = msg.data.split('h')
            minutes = minutes.strip().split('m')[0]
            hours = hours.strip()

            write_data = {
                "hour": hours,
                "min": minutes
            }

            with open('user_data/bat_time.json', 'w') as d:
                json.dump(write_data, d)
        except ValueError:
            pass

    def subscribe_callback_co(self, msg):
        lang = language_check()
        if msg.data >= 200 and lang == 1:
            speaking("불이 났어요!!")
        elif msg.data >= 200 and lang == 0:
            speaking_en("Fire!!!")


def conversation_loop(talking_node):
    print("conversation_loop() korean 시작")
    mj = MYOUNGJA()
    
    # Clean up before starting the loop
    file_cleanup()

    name_check()

    # Start of the conversation
    # Initialize the audio stream with sounddevice
    stream = sd.InputStream(
        samplerate=RATE, channels=CHANNELS, dtype='int16')
    stream.start()

    owwModel = Model(
        wakeword_models=["./src/mjbot_voice/models/hi.tflite"], inference_framework="tflite")

    n_models = len(owwModel.models.keys())

    # Main loop for wake word detection
    while True:
        # Get audio
        audio_data, overflowed = stream.read(CHUNK)
        if overflowed:
            print("Audio buffer has overflowed")

        audio_data = np.frombuffer(audio_data, dtype=np.int16)

        # Feed to openWakeWord model
        prediction = owwModel.predict(audio_data)
        common = False
        # Process prediction results
        for mdl in owwModel.prediction_buffer.keys():
            scores = list(owwModel.prediction_buffer[mdl])
            if scores[-1] > 0.2:  # Wake word detected
                print(f"Wake word detected !!!!!!!!!!!!!!!!1 {mdl}!")
                mdl = ""
                scores = [0] * n_models
                audio_data = np.array([])
                common = True
        if common:
            use_sound("./mp3/yes.wav")
            common = False

            while True:
                # 대답 기다리는 동안 표정 변화
                talking_node.publish_emotions("mic_waiting")
                response = mic(3)
                talking_node.publish_emotions("daily")

                if response == "초기화":
                    use_sound("./mp3/reset.wav")
                    name_ini()
                elif response == "종료":
                    use_sound("./mp3/off.wav")
                    return 3
                elif response == "조용":
                    use_sound("./mp3/quiet.wav")
                    # call_num = - 1000000
                elif response == "외국어":
                    speaking_en("English mode")
                    langugage_change(False)
                    print("english change")
                    return 2
                # 복약 기능 추가
                elif response == "복약":
                    disease_alarm()
                elif response == "시간":
                    time_alarm()
                elif response == "배터리":
                    with open('user_data/bat_percent.json', 'r') as f:
                        data = json.load(f)
                        bat_state = data["bat_state"]

                    # 남은 사용 가능 시간 체크
                    with open('user_data/bat_time.json', 'r') as f:
                        data = json.load(f)
                        use_hour = data["hour"]
                        use_min = data["min"]
                    speaking(
                        f"배터리 잔량은 {bat_state} 퍼센트 입니다. 남은 사용 시간은 {use_hour}시간 {use_min}분 남았습니다.")

                elif response != "":
                    print("other response")
                    if response == "산책 가자":  # 산책 가자
                        talking_node.publish_arm_motions("holding_hand")
                        speaking("좋아요 산책 가요")
                        time.sleep(1)
                    elif response == "다나와":  # 따라와
                        talking_node.publish_mode("tracking")
                        use_sound("./mp3/yes.wav")
                        speaking("따라가기 모드 시작합니다")
                    elif response == "따라와":  # 따라와
                        talking_node.publish_mode("tracking")
                        use_sound("./mp3/yes.wav")
                        speaking("따라가기 모드 시작합니다")
                    elif response == "멈춰":  # 멈춰
                        talking_node.publish_mode("idle")
                        talking_node.publish_arm_motions("default")
                        use_sound("./mp3/yes.wav")
                    elif response == "오른손":  # 오른손
                        talking_node.publish_arm_motions("give_right_hand")
                        use_sound("./mp3/yes.wav")
                        speaking("오른손")
                    elif response == "왼손":
                        talking_node.publish_arm_motions("give_left_hand")
                        use_sound("./mp3/yes.wav")
                        speaking("왼손")
                    elif response == "안아줘":
                        talking_node.publish_arm_motions("hug")
                        use_sound("./mp3/yes.wav")
                        speaking("안아드릴께요")
                    elif response == "조용":
                        break
                    else:
                        print("Chat gpt start")
                        response_ = mj.gpt_send_anw(response)
                        print(response_)
                        emotion = response_[0]

                        # React based on the emotion
                        if emotion == "close":
                            talking_node.publish_emotions("close")
                        elif emotion == "moving":
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
                elif response == "":
                    return 1
            

def conversation_loop_en(talking_node):
    print("conversation_loop() english 시작")
    mj = MYOUNGJA()

    # Clean up before starting the loop
    file_cleanup()

    name_check()

    # 배터리 잔량 체크

    # Experimental feature to initiate conversation
    # if not common:
    #     use_sound("./mp3/ex1.wav")
    #     time.sleep(2)
    #     use_sound("./mp3/ex_2.wav")
    #     common = True

    # Start of the conversation
    # Initialize the audio stream with sounddevice
    stream = sd.InputStream(
        samplerate=RATE, channels=CHANNELS, dtype='int16')
    stream.start()

    owwModel = Model(
        wakeword_models=["./src/mjbot_voice/models/hi.tflite"], inference_framework="tflite")

    n_models = len(owwModel.models.keys())

    # Main loop for wake word detection
    while True:
        # Get audio
        audio_data, overflowed = stream.read(CHUNK)
        if overflowed:
            print("Audio buffer has overflowed")

        audio_data = np.frombuffer(audio_data, dtype=np.int16)

        # Feed to openWakeWord model
        prediction = owwModel.predict(audio_data)
        common = False
        # Process prediction results
        for mdl in owwModel.prediction_buffer.keys():
            scores = list(owwModel.prediction_buffer[mdl])
            if scores[-1] > 0.2:  # Wake word detected
                print(f"Wake word detected !!!!!!!!!!!!!!!!1 {mdl}!")
                mdl = ""
                scores = [0] * n_models
                audio_data = np.array([])
                common = True
        if common:
            speaking_en("Yes sir!!")
            common = False

            while True:
                # 대답 기다리는 동안 표정 변화
                talking_node.publish_emotions("mic_waiting")
                response = mic_en(3)
                talking_node.publish_emotions("daily")

                if response == "reset":
                    speaking_en("Ok reset mode")
                    name_ini()
                elif response == "turn off":
                    speaking_en("ok turn off mode")
                    return 3
                elif response == "silent":
                    speaking_en("ok silent mode")
                    # call_num = - 1000000
                elif response == "change":
                    speaking("한국어 모드로 전환합니다")
                    langugage_change(True)
                    return 1
                elif response == "battery":
                    with open('user_data/bat_percent.json', 'r') as f:
                        data = json.load(f)
                        bat_state = data["bat_state"]

                    # 남은 사용 가능 시간 체크
                    with open('user_data/bat_time.json', 'r') as f:
                        data = json.load(f)
                        use_hour = data["hour"]
                        use_min = data["min"]
                    speaking_en(
                        f"The battery level is {bat_state} percent. The remaining usage time is {use_hour} hours and {use_min} minutes.")

                elif response != "":
                    if response == "Let's take a walk":  # 산책 가자
                        talking_node.publish_arm_motions("holding_hand")
                        speaking_en("Yes sir!!")
                        time.sleep(1)
                    elif response == "follow me":  # 따라와
                        talking_node.publish_mode("tracking")
                        speaking_en("Yes sir!!")
                    elif response == "stop":  # 멈춰
                        talking_node.publish_mode("idle")
                        speaking_en("Yes sir!!")
                    elif response == "right hand":  # 오른손
                        talking_node.publish_arm_motions("give_right_hand")
                        speaking_en("Yes sir!!")
                    elif response == "left hand":
                        talking_node.publish_arm_motions("give_left_hand")
                        speaking_en("Yes sir!!")
                    elif response == "hug me":
                        talking_node.publish_arm_motions("hug")
                        speaking_en("Yes sir!!")
                    elif response == "silent":
                        break
                    else:

                        response_ = mj.gpt_send_anw(response)
                        emotion = response_[0]

                        # React based on the emotion
                        if emotion == "close":
                            talking_node.publish_emotions("close")
                        elif emotion == "moving":
                            talking_node.publish_emotions("moving")
                        elif emotion == "angry":
                            talking_node.publish_emotions("angry")
                        elif emotion == "sad":
                            talking_node.publish_emotions("sad")
                        else:
                            talking_node.publish_emotions("daily")

                        ans = response_[1]
                        speaking_en(ans)
                        talking_node.publish_emotions("daily")
                elif response == "":
                    return 1


def start_executor_thread(executor):
    """Start a threaded execution of ROS nodes."""
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    return executor_thread


def main():
    """Main function to initialize and run the ROS nodes."""
    rclpy.init(args=None)

    talking_node = TalkingNode()
    hear_node = VoiceSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(talking_node)
    executor.add_node(hear_node)

    executor_thread = start_executor_thread(executor)

    mode = 1
    try:
        while True:
            if mode == 1:
                speaking("안녕하세요!!")
            elif mode == 2:
                speaking_en("Hello! sir!!")
            while True:
                print(f"mode is {mode}")
                if mode == 1:
                    KR = conversation_loop(talking_node)
                    file_cleanup()
                    if KR == 2:
                        mode = 2
                        break
                elif mode == 2:
                    EN = conversation_loop_en(talking_node)
                    file_cleanup()
                    if EN == 1:
                        mode = 1
                        break
                else:
                    break

    except KeyboardInterrupt:
        pass
    finally:
        if executor_thread.is_alive():
            executor_thread.join()
        talking_node.destroy_node()
        hear_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
