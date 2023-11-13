#!/usr/bin/env python3
import openwakeword
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int32

import threading
import json
import os
import time
import pyaudio

# Assuming these functions are correctly implemented in the Chat.voiceChat module
from Chat.voiceChat import speaking, use_sound, mic, name_check, name_ini

# Global variables for file names
SAMPLE_WAV = "./sampleWav.wav"
RESULT_MP3 = "./ResultMP3.mp3"
YES_WAV = "./yes.wav"
BAT_VALUE_JSON = './bat_value.json'

# Helper function to clean up audio files


def init_wake_word():
    wake_word_engine = openwakeword.create(
        model_path="./src/mjbot_voice/models/openwakeword_model.pmdl",  # example model path
        sensitivity=0.5  # example sensitivity setting
    )
    pa = pyaudio.PyAudio()
    audio_stream = pa.open(
        rate=wake_word_engine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=wake_word_engine.frame_length
    )
    return wake_word_engine, pa, audio_stream


def cleanup_wake_word(wake_word_engine, pa, audio_stream):
    if wake_word_engine is not None:
        wake_word_engine.delete()
    if audio_stream is not None:
        audio_stream.close()
    if pa is not None:
        pa.terminate()


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


def wake_word_loop(talking_node, wake_word_engine, audio_stream):
    print("Waiting for the wake word...")
    while rclpy.ok():
        pcm = audio_stream.read(
            wake_word_engine.frame_length, exception_on_overflow=False)
        is_wake_word_detected = wake_word_engine.process(pcm)

        if is_wake_word_detected:
            print("Wake word detected!")
            conversation_loop(talking_node)
            # If you want to continue listening for the wake word after a conversation, do not break here


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
        self.create_subscription(
            Bool, 'owner_fall', self.callback_fall_down, 10)
        self.create_subscription(
            String, 'bat_percent', self.callback_bat_state, 10)
        self.create_subscription(Bool, 'touch', self.callback_touch, 10)
        self.create_subscription(Int32, 'co_ppm', self.callback_co, 10)

    def callback_fall_down(self, msg):
        if msg.data:
            speaking("할머니 괜찮으세요??")

    def callback_bat_state(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        bat_state, bat_hour, bat_min = parse_battery_status(msg.data)
        save_battery_status(bat_state, bat_hour, bat_min)
        if int(bat_state) <= 40:
            speaking("할머니 배고파요")

    def callback_touch(self, msg):
        if msg.data:
            speaking("깔깔깔")

    def callback_co(self, msg):
        if msg.data >= 200:
            speaking("할머니 불이 났어요!!")


def conversation_loop(talking_node):
    call_num = 0
    common = False

    # Clean up before starting the loop
    file_cleanup()

    while rclpy.ok():  # Use rclpy.ok() to check for shutdown signals
        name_check()

        # Load battery status
        with open('./bat_value.json', 'r') as f:
            data = json.load(f)
        bat_state = data["bat_state"]
        bat_hour = data["bat_hour"]
        bat_min = data["bat_min"]

        # Experimental feature to initiate conversation
        if not common:
            use_sound("./mp3/ex1.wav")
            time.sleep(2)
            use_sound("./mp3/ex_2.wav")
            common = True

        # Start of the conversation
        response = mic(2)
        if response == "":
            call_num += 1
            if call_num == 3:
                use_sound("./mp3/say_my_name.wav")
                call_num = 0
        else:
            if response == "로봇":
                use_sound("./mp3/yes.wav")
                talking_node.publish_emotions("mic_waiting")
                response = mic(3)
                talking_node.publish_emotions("daily")

                # Process the response
                if response == "초기화":
                    use_sound("./mp3/reset.wav")
                    name_ini()
                elif response == "종료":
                    use_sound("./mp3/off.wav")
                    break  # Exit the loop to shutdown
                elif response == "조용":
                    use_sound("./mp3/quiet.wav")
                    call_num = -1000000  # Effectively disables the call response
                elif response == "배터리":
                    speaking(
                        f"배터리 잔량은 {bat_state}%입니다. 남은 사용 시간은 {bat_hour}시간 {bat_min}분입니다.")
                elif response == "따라와":
                    talking_node.publish_arm_motions("tracking")
                elif response == "멈춰":
                    talking_node.publish_arm_motions("idle")
                elif response == "오른손":
                    talking_node.publish_arm_motions("give_right_hand")
                elif response == "왼손":
                    talking_node.publish_arm_motions("give_left_hand")
                elif response == "안아줘":
                    talking_node.publish_arm_motions("hug")
                elif response == "조용":
                    break
                else:
                    # Assuming `mj.gpt_send_anw` is a function that returns a tuple (emotion, response)
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

        # If no response, clean up and prepare for the next iteration
        if response == "":
            file_cleanup()

    # Clean up and shutdown after breaking out of the loop
    file_cleanup()
    talking_node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    talking_node = TalkingNode()
    hear_node = VoiceSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(talking_node)
    executor.add_node(hear_node)

    wake_word_engine, pa, audio_stream = init_wake_word()

    executor_thread = start_executor_thread(executor)

    try:
        wake_word_loop(talking_node, wake_word_engine, audio_stream)
    finally:
        cleanup_wake_word(wake_word_engine, pa, audio_stream)
        if executor_thread.is_alive():
            executor_thread.join()
        talking_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
