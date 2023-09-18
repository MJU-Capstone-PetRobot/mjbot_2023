#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

import serial
import threading
import time

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
        self.publisher_bat_state_ = self.create_publisher(String, "bat", 10)
        self.get_logger().info("Node has been started")
        
    # 발행 : 배터리(string), 터치(bool), 일산화탄소(int), 거리(int)
    # 구독 : 목각도(string), 감정(int)
    def publish_bat(self, bat): 
        msg = String()
        msg.data = bat
        self.publisher_bat_state_.publish(msg)
        self.get_logger().info("PUB: /bat: {}".format(msg.data))

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
                    elif esp_packet[1] == 'C':
                        co_ppm = esp_packet[2:-1]
                        msg += "[CO_PPM] " + co_ppm + "ppm"
                    elif esp_packet[1] == 'D':
                        distance = esp_packet[2:-1]
                        msg += "[DISTANCE] " + distance + "mm"
                    elif esp_packet[1] == 'B':
                        bat_state = esp_packet[2:-1]
                        msg += "[BAT] " + bat_state
                    else:
                        msg += "[ERROR]"

                    print(msg)
                else:
                    print(esp_packet)
                esp_packet = ""

    SerialObj.close()

def main(args=None):
    thread = threading.Thread(target=receive_from_esp, args=(SerialObj,))
    thread.start()

    rclpy.init(args=args)
    node = OpiEspNode()

    while True:
        rclpy.spin_once(node, timeout_sec=0)

        opi_packet = "(E3)"
        SerialObj.write(opi_packet.encode())
        time.sleep(1)

        opi_packet = "(N1,5,80,1)"
        SerialObj.write(opi_packet.encode())
        time.sleep(1)

        # node.publish_bat(bat_state)

    rclpy.shutdown()
    esp_thread = False

if __name__ == "__main__": 
    main() 