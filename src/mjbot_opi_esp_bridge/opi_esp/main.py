#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import UInt16

from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3

import serial
import threading

port = '/dev/ttyUSB0' 
baud = 115200 

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

        self.publisher_ultrasonic_1_ = self.create_publisher(Range, "ultrasonic_1", 10)
        self.publisher_ultrasonic_2_ = self.create_publisher(Range, "ultrasonic_2", 10)
        self.publisher_bat_state_ = self.create_publisher(String, "bat", 10)
        self.publisher_touch_ = self.create_publisher(Bool, "touch", 10)
        self.publisher_co_ = self.create_publisher(Int32, "co_ppm", 10)

        self.subscriber_emo = self.create_subscription(String, "emo", self.callback_emo, 10)
        self.subscriber_neck_rpy = self.create_subscription(Vector3, "neck_rpy", self.callback_neck_rpy, 10)
        self.subscriber_neck_z = self.create_subscription(UInt16, "neck_z", self.callback_neck_z, 10)

        self.get_logger().info("opi_esp_comm node has been started")
        
    # 발행 : 배터리(string), 터치(bool), 일산화탄소(int), 거리(int)
    # 구독 : 목각도(string), 감정(string)
    def publish_bat(self, bat): 
        msg = String()
        msg.data = bat
        self.publisher_bat_state_.publish(msg)
        self.get_logger().info("[PUB] /bat [{}]".format(msg.data))

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

    def publisher_ultrasonic(self, ultra_1, ultra_2): 
        msg_1 = Range()
        msg_1.radiation_type = 0 # ULTRASOUND
        msg_1.field_of_view = 1.0472 # radian, 60 degree = 1.0472 radian
        msg_1.min_range = 0.030 # m
        msg_1.max_range = 4.500 # m
        msg_1.range = ultra_1 / 1000 # mm -> m

        msg_2 = Range()
        msg_2.radiation_type = 0 # ULTRASOUND
        msg_2.field_of_view = 1.0472 # radian, 60 degree = 1.0472 radian
        msg_2.min_range = 0.030 # m
        msg_2.max_range = 4.500 # m
        msg_2.range = ultra_2 / 1000 # mm -> m

        self.publisher_ultrasonic_1_.publish(msg_1)
        self.publisher_ultrasonic_2_.publish(msg_2)

        self.get_logger().info("[PUB] /ultrasonic_1, 2 [{}] [{}]".format(msg_1.range, msg_2.range))   

    def callback_emo(self, sub_msg):
        self.get_logger().info("[SUB] /emo: [{}]".format(sub_msg.data))
        self.emo = sub_msg.data

        opi_packet = "(E" + self.emo + ")"
        SerialObj.write(opi_packet.encode())

        opi_packet = ''

    def callback_neck_rpy(self, sub_msg):
        self.get_logger().info("[SUB] /neck_rpy: [{}][{}][{}]".format(sub_msg.x, sub_msg.y, sub_msg.z))
        self.neck[0] = sub_msg.x
        self.neck[1] = sub_msg.y
        self.neck[3] = sub_msg.z

        opi_packet = '(N' + str(self.neck) + ')'
        SerialObj.write(opi_packet.encode())

        opi_packet = ''

    def callback_neck_z(self, sub_msg):
        self.get_logger().info("[SUB] /neck_z: [{}]".format(sub_msg.data))
        self.neck[2] = sub_msg.data

        opi_packet = '(N' + str(self.neck) + ')'
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

                if esp_packet[0] == '<' and esp_packet[len(esp_packet) -1] == '>':
 
                    if esp_packet[1] == 'T':
                        touch = esp_packet[2]
                        node.publisher_touch(bool(touch))
                    elif esp_packet[1] == 'C':
                        co_ppm = esp_packet[2:-1]
                        node.publisher_co(int(co_ppm))
                    elif esp_packet[1] == 'D':
                        distance = esp_packet[2:-1]
                        node.publisher_ultrasonic(int(distance), int(distance))
                    elif esp_packet[1] == 'B':
                        bat_state = esp_packet[2:-1]
                        node.publish_bat(bat_state)
                    else:
                        print("esp packet not defined")

                esp_packet = ""

    SerialObj.close()

def main(args=None):
    rclpy.init(args=args)
    global node
    node = OpiEspNode()

    serial_thread = threading.Thread(target=receive_from_esp, args=(SerialObj,))
    serial_thread.start()

    while True:
        rclpy.spin_once(node, timeout_sec=0)


    rclpy.shutdown()
    esp_thread = False

if __name__ == "__main__": 
    main() 