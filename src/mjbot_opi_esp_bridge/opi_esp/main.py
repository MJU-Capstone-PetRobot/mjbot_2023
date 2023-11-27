#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import UInt16

from sensor_msgs.msg import Range
from geometry_msgs.msg import Quaternion
import time
import serial
import threading
from rclpy.executors import MultiThreadedExecutor

port = '/dev/esp'
baud = 1000000

SerialObj = serial.Serial(port, baud, timeout=3)
esp_thread = True

esp_packet = ""
opi_packet = ""

touch = ""
co_ppm = ""
distance = ""
bat_state = ""


class OpiEspNode(Node):
    def __init__(self):
        super().__init__("opi_esp_comm")
        self.emo = ''

        self.neck = [0, 0, 0, 0]

        # 발행
        self.publisher_ultrasonic_1_ = self.create_publisher(
            Range, "ultrasonic_1", 10)
        self.publisher_ultrasonic_2_ = self.create_publisher(
            Range, "ultrasonic_2", 10)

        self.publisher_bat_percent_ = self.create_publisher(
            String, "bat_percent", 10)
        self.publisher_bat_time_ = self.create_publisher(
            String, "bat_time", 10)

        self.publisher_touch_ = self.create_publisher(Bool, "touch", 10)
        self.publisher_co_ = self.create_publisher(Int32, "co_ppm", 10)

        self.publisher_gps_ = self.create_publisher(String, "gps", 10)

        # 구독
        self.subscriber_emo = self.create_subscription(
            String, "emo", self.callback_emo, 10)
        self.subscriber_neck_rpy = self.create_subscription(

            Quaternion, "neck_rpyz", self.callback_neck_rpy, 10)

        self.get_logger().info("opi_esp_comm node has been started")

    # 발행 : 초음파 센서 1(Range), 초음파 센서 2(Range),
    #       배터리 잔량(String), 배터리 지속 시간(String),
    #       터치(Bool), 일산화탄소(Int32), GPS(String)
    # 구독 : 표정(String), 목각도 RPZ(Vector3)
    def publisher_ultrasonic(self, ultra_1, ultra_2):
        int(ultra_1)
        int(ultra_2)
        msg_1 = Range()
        msg_1.radiation_type = 0  # ULTRASOUND
        msg_1.field_of_view = 1.0472  # radian, 60 degree = 1.0472 radian
        msg_1.min_range = 0.030  # m
        msg_1.max_range = 4.500  # m
        msg_1.range = ultra_1 / 1000  # mm -> m

        msg_2 = Range()
        msg_2.radiation_type = 0  # ULTRASOUND
        msg_2.field_of_view = 1.0472  # radian, 60 degree = 1.0472 radian
        msg_2.min_range = 0.030  # m
        msg_2.max_range = 4.500  # m
        msg_2.range = ultra_2 / 1000  # mm -> m

        self.publisher_ultrasonic_1_.publish(msg_1)
        self.publisher_ultrasonic_2_.publish(msg_2)

        # self.get_logger().info(
        #     "[PUB] /ultrasonic_1, 2 [{}] [{}]".format(msg_1.range, msg_2.range))

    def publish_bat_percent(self, bat_percent):
        msg = String()
        msg.data = bat_percent
        self.publisher_bat_percent_.publish(msg)
        self.get_logger().info("[PUB] /bat_percent [{}]".format(msg.data))

    def publish_bat_time(self, bat_time):
        msg = String()
        msg.data = bat_time
        self.publisher_bat_time_.publish(msg)
        self.get_logger().info("[PUB] /bat_time [{}]".format(msg.data))

    def publisher_touch(self, touch):
        msg = Bool()
        msg.data = touch
        self.publisher_touch_.publish(msg)
        self.get_logger().info("[PUB] /touch [{}]".format(msg.data))

    def publisher_co(self, co_ppm):
        msg = Int32()
        msg.data = co_ppm
        self.publisher_co_.publish(msg)
        self.get_logger().info("[PUB] /co_ppm [{}]".format(msg.data))

    def publish_gps(self, gps):
        msg = String()
        msg.data = gps
        self.publisher_gps_.publish(msg)
        self.get_logger().info("[PUB] /gps [{}]".format(msg.data))

    def callback_emo(self, sub_msg):
        self.get_logger().info("[SUB] /emo: [{}]".format(sub_msg.data))
        self.emo = sub_msg.data

        opi_packet = "(E^" + self.emo + ")"
        time.sleep(0.3)
        SerialObj.write(opi_packet.encode())

        opi_packet = ''

    def callback_neck_rpy(self, sub_msg):

        self.get_logger().info(
            "[SUB] /neck_rpyz: [{}][{}][{}][{}]".format(sub_msg.x, sub_msg.y, sub_msg.z, sub_msg.w))

        self.neck[0] = sub_msg.x
        self.neck[1] = sub_msg.y
        self.neck[2] = sub_msg.z
        self.neck[3] = sub_msg.w

        opi_packet = '(N^' + str(self.neck[0])+',' + str(
            self.neck[1])+','+str(self.neck[2])+','+str(self.neck[3]) + ')'
        SerialObj.write(opi_packet.encode())
        opi_packet = ''


def receive_from_esp(SerialObj):
    global esp_packet
    msg = ''

    while esp_thread:
        for c in SerialObj.read():
            esp_packet += (chr(c))
            if esp_packet.endswith('\n'):
                esp_packet = esp_packet.strip()

                if esp_packet[0] == '<' and esp_packet[len(esp_packet) - 1] == '>':
                    if esp_packet[1] == 'T':
                        if esp_packet[3] == '0':
                            touch = False
                        elif esp_packet[3] == '1':
                            touch = True
                        node.publisher_touch(bool(touch))
                    elif esp_packet[1] == 'C':
                        co_ppm = esp_packet[3:-1]
                        node.publisher_co(int(co_ppm))
                    elif esp_packet[1] == 'D':
                        end_index = esp_packet.find(',')
                        distance1 = esp_packet[3:end_index]

                        start_index = end_index + 1
                        distance2 = esp_packet[start_index:-1]

                        try:
                            # Attempt to parse the distances as floats
                            node.publisher_ultrasonic(
                                float(distance1), float(distance2))
                        except ValueError:
                            node.get_logger().error("Invalid distance data received:")
                    elif esp_packet[1] == 'B' and esp_packet[2] == 'D':
                        bat_time = esp_packet[4:-1]
                        node.publish_bat_time(bat_time)
                    elif esp_packet[1] == 'B':
                        bat_percent = esp_packet[3:-1]
                        node.publish_bat_percent(bat_percent)
                    elif esp_packet[1] == 'G':
                        gps = esp_packet[3:-1]
                        node.publish_gps(gps)
                    else:
                        print("esp packet not defined")

                esp_packet = ""

    SerialObj.close()


def main(args=None):
    rclpy.init(args=args)
    global node
    node = OpiEspNode()
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    serial_thread = threading.Thread(
        target=receive_from_esp, args=(SerialObj,))
    serial_thread.start()

    rate = node.create_rate(10)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    esp_thread = False


if __name__ == "__main__":
    main()
