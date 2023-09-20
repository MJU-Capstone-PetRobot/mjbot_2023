#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
from example_interfaces.msg import Bool
from example_interfaces.msg import Int32
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16

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

        self.publisher_bat_state_ = self.create_publisher(String, "bat", 10)
        self.publisher_touch_ = self.create_publisher(Bool, "touch", 10)
        self.publisher_co_ = self.create_publisher(Int32, "co_ppm", 10)
        self.publisher_distance_ = self.create_publisher(Int32, "distance", 10)

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
        self.get_logger().info("[PUB] /bat: {}".format(msg.data))

    def publisher_touch(self, touch): 
        msg = Bool()
        msg.data = touch
        self.publisher_touch_.publish(msg)
        self.get_logger().info("[PUB] /touch: {}".format(msg.data))

    def publisher_co(self, co_ppm): 
        msg = Int32()
        msg.data = co_ppm
        self.publisher_co_.publish(msg)
        self.get_logger().info("[PUB] /co_ppm: {}".format(msg.data))

    def publisher_distance(self, dist): 
        msg = Int32()
        msg.data = dist
        self.publisher_distance_.publish(msg)
        self.get_logger().info("[PUB] /distance: {}".format(msg.data))   

    def callback_emo(self, sub_msg):
        self.get_logger().info("[SUB] /emo: {}".format(sub_msg.data))
        self.emo = sub_msg.data

        opi_packet = "(E" + self.emo + ")"
        SerialObj.write(opi_packet.encode())

        opi_packet = ''

    def callback_neck_rpy(self, sub_msg):
        self.get_logger().info("[SUB] /neck_rpy: {} {} {}".format(sub_msg.x, sub_msg.y, sub_msg.z))
        self.neck[0] = sub_msg.x
        self.neck[1] = sub_msg.y
        self.neck[3] = sub_msg.z

        opi_packet = '(N' + str(self.neck) + ')'
        SerialObj.write(opi_packet.encode())

        opi_packet = ''

    def callback_neck_z(self, sub_msg):
        self.get_logger().info("[SUB] /neck_z: {}".format(sub_msg.data))
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
                    msg = ""
                    msg = "[ESP->OPI] "
                    msg += esp_packet + " "

                    if esp_packet[1] == 'T':
                        touch = esp_packet[2]
                        msg += "[TOUCH] " + touch
                        node.publisher_touch(bool(touch))
                    elif esp_packet[1] == 'C':
                        co_ppm = esp_packet[2:-1]
                        msg += "[CO_PPM] " + co_ppm + "ppm"
                        node.publisher_co(int(co_ppm))
                    elif esp_packet[1] == 'D':
                        distance = esp_packet[2:-1]
                        msg += "[DISTANCE] " + distance + "mm"
                        node.publisher_distance(int(distance))
                    elif esp_packet[1] == 'B':
                        bat_state = esp_packet[2:-1]
                        msg += "[BAT] " + bat_state
                        node.publish_bat(bat_state)
                    else:
                        msg += "[ERROR]"

                    print(msg)
                else:
                    print(esp_packet)
                esp_packet = ""

    SerialObj.close()

def main(args=None):
    rclpy.init(args=args)
    global node
    node = OpiEspNode()

    thread = threading.Thread(target=receive_from_esp, args=(SerialObj,))
    thread.start()

    while True:
        rclpy.spin_once(node, timeout_sec=0)

        # opi_packet = "(E3)"
        # SerialObj.write(opi_packet.encode())
        # time.sleep(1)

        # opi_packet = "(N1,5,80,1)"
        # SerialObj.write(opi_packet.encode())
        # time.sleep(1)

        # node.publish_bat(bat_state)

    rclpy.shutdown()
    esp_thread = False

if __name__ == "__main__": 
    main() 