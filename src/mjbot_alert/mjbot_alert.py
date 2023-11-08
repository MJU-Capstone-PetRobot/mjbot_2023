#!/usr/bin/python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from Alert.message_send import *


class ListeningNode(Node):
    def __init__(self):
        super().__init__('Listening_node')
        self.subscription = self.create_subscription(
            String, 'gps', self.subscribe_callback_gps, 10)
        self.subscription = self.create_subscription(
            Int32, 'co_ppm', self.subscribe_callback_fire, 10)


    def subscribe_callback_fire(self, msg):

        self.get_logger().info('Received: %s' % msg.data)

        if msg.data >= 200:
            send_message(2)  # 화재 사고 발생 문자 발송
            time.sleep(1000)

    def subscribe_callback_gps(self, msg):
        import json
        self.get_logger().info('Received: %s' % msg.data)

        data_ = list(msg.data)

        lat = []
        lon = []
        j = 0
        for i in range(0, len(data_)):
            if data_[i] == ',':
                j = i
                break
            else:
                lat.append(data_[i])
        for j in range(j, len(data_)):
            if data_[j] == ' ':
                pass
            elif data_[j] == ",":
                pass
            else:
                lon.append(data_[j])

        latitude_str = "".join(lat)
        longitude_str = "".join(lon)

        write_data = {
            "lat": f"{latitude_str}",
            "lon": f"{longitude_str}"
        }
        with open('./user_gps.json', 'w') as d:
            json.dump(write_data, d)


def main(args=None):
    rclpy.init(args=args)
    Listening_node = ListeningNode()
    rclpy.spin(Listening_node)

    Listening_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
