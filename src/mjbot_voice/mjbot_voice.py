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
BAT_VALUE_JSON = 'user_data/bat_value.json'

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


def parse_battery_status(bat_status):
    bat_state, bat_hour, bat_min = bat_status[:
                                              2], bat_status[3], bat_status[4:-1]
    return {
        "state": int(''.join(bat_state)),
        "hour": ''.join(bat_hour),
        "minute": ''.join(bat_min)
    }

# Helper function to save battery status to JSON


def save_battery_status(bat_state, bat_hour, bat_min):
    write_data = {
        "bat_state": bat_state,
        "bat_hour": bat_hour,
        "bat_min": bat_min
    }
    with open(BAT_VALUE_JSON, 'w') as file:
        json.dump(write_data, file)

# Helper function for threaded execution of ROS nodes


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

        if msg.data == True:
            speaking("할머니 괜찮으세요??")

    def subscribe_callback_bat_state(self, msg):
        import json
        self.get_logger().info('Received: %s' % msg.data)

        bat_state = float(msg.data)
        write_data = {
            "bat_state": bat_state,
        }

        with open('user_data/bat_time.json', 'w') as d:
            json.dump(write_data, d)

        if bat_state <= 40.0:
            speaking("할머니 배고파요")

    def subscribe_callback_bat_time(self, msg):
        import json
        self.get_logger().info('Received: %s' % msg.data)

        bat_time = list(msg.data)
        hour = []
        min = []
        j = 0
        for i in range(0, len(bat_time)):
            if bat_time[i] == 'h':
                j = i
            hour.append(bat_time[i])
        for k in range(j+1, len(bat_time)):
            if bat_time[k] == ' ':
                continue
            elif bat_time[k] == 'm':
                break
            else:
                min.append(bat_time[k])
        hour_ = "".join(hour)
        min_ = "".join(min)

        write_data = {
            "hour": hour_,
            "min": min_
        }

        with open('user_data/bat_time.json', 'w') as d:
            json.dump(write_data, d)

    def subscribe_callback_co(self, msg):
        if msg.data >= 200:
            speaking("할머니 불이 났어요!!")


def conversation_loop(talking_node):
    call_num = 0
    common = False
    mj = MYOUNGJA()

    # Clean up before starting the loop
    file_cleanup()

    name_check()

    # 배터리 잔량 체크
    with open('user_data/bat_percent.json', 'r') as f:
        data = json.load(f)
        bat_state = data["bat_state"]

    # 남은 사용 가능 시간 체크
    with open('user_data/bat_time.json', 'r') as f:
        data = json.load(f)
        use_hour = data["hour"]
        use_min = data["min"]

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
        wakeword_models=["./src/mjbot_voice/models/hey.tflite"], inference_framework="tflite")

    n_models = len(owwModel.models.keys())

    # Main loop for wake word detection
    try:
        while True:

            # Get audio
            audio_data, overflowed = stream.read(CHUNK)
            if overflowed:
                print("Audio buffer has overflowed")

            audio_data = np.frombuffer(audio_data, dtype=np.int16)

            # Feed to openWakeWord model
            prediction = owwModel.predict(audio_data)

            # Process prediction results
            for mdl in owwModel.prediction_buffer.keys():
                scores = list(owwModel.prediction_buffer[mdl])
                if scores[-1] > 0.4:  # Wake word detected
                    print(f"Wake word detected !!!!!!!!!!!!!!!!1 {mdl}!")
                    mdl = ""
                    scores = [0] * n_models
                    audio_data = np.array([])

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
                        speaking(
                            f"배터리 잔량은 {bat_state} 퍼센트 입니다. 남은 사용 시간은 {use_hour}시간 {use_min}분 남았습니다.")

                    if response != "":
                        if response == "산책 가자":  # 산책 가자
                            talking_node.publish_arm_motions("holding_hand")
                            time.sleep(1)
                        elif response == "따라와":  # 따라와
                            talking_node.publish_mode("tracking")
                        elif response == "멈춰":  # 멈춰
                            talking_node.publish_mode("idle")
                        elif response == "오른손":  # 오른손
                            talking_node.publish_arm_motions("give_right_hand")
                        elif response == "왼손":
                            talking_node.publish_arm_motions("give_left_hand")
                        elif response == "안아줘":
                            talking_node.publish_arm_motions("hug")
                        elif response == "조용":
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
                            speaking(ans)
                            talking_node.publish_emotions("daily")

                        # Remove temporary files after processing each response
                    file_cleanup()
                    response = ""
                    break
                break

    finally:
        # Clean up
        stream.stop()
        stream.close()


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

    try:
        while True:
            conversation_loop(talking_node)
    finally:
        if executor_thread.is_alive():
            executor_thread.join()
        talking_node.destroy_node()
        hear_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
