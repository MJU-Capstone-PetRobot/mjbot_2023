#!/usr/bin/python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from Alert.message_send import *
import threading

class SpeakingNode(Node):
    def __init__(self):
        super().__init__('speaking_node')
        self.get_logger().info("Speaking Node")
        self.publisher_danger = self.create_publisher(
            Bool, 'danger', 10)

    def publish_danger(self, danger):
        msg = Bool()
        msg.data = danger
        self.publisher_danger.publish(msg)
        self.get_logger().info("[PUB] /danger [{}]".format(msg.data))

class ListeningNode(Node):
    def __init__(self):
        super().__init__('Listening_node')
        self.subscription = self.create_subscription(
            String, 'gps', self.subscribe_callback_gps, 10)
        self.subscription = self.create_subscription(
            Int32, 'co_ppm', self.subscribe_callback_fire, 10)
        self.subscription = self.create_subscription(
            Bool, 'owner_fall', self.subscribe_callback_fall, 10)


    def subscribe_callback_fire(self, msg):
        import json

        self.get_logger().info('Received: %s' % msg.data)

        if msg.data >= 200:
            danger_write(1)
            send_message(2)  # 화재 사고 발생 문자 발송
            time.sleep(100)
            danger_write(2)


    def subscribe_callback_fall(self, msg):

        self.get_logger().info('Received: %s' % msg.data)

        if msg.data == 1:
            danger_write(1)
            time.sleep(100)
            danger_write(0)

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
        with open('/home/mju/mjbot_2023/user_data/user_gps.json', 'w') as d:
            json.dump(write_data, d)

def main(args=None):
    import json
    rclpy.init(args=args)
    publish_node = SpeakingNode()
    listen_node = ListeningNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publish_node)
    executor.add_node(listen_node)

    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    danger_write(2)
    while(1):
        if danger_check() == 1:
            publish_node.publish_danger(True)
            time.sleep(10)
            danger_write(2)
            break

    executor_thread.join()
    publish_node.destroy_node()
    listen_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
