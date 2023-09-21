#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from example_interfaces.msg import Bool
from example_interfaces.msg import Int16MultiArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16

cam_center = 320 # 640 X 360

class FakeCmdNode(Node):
    def __init__(self):
        super().__init__("fake_cmd")
        self.emo = 0
        self.owner_exists = False
        self.owner_size = [0 ,0] # [w, h]
        self.owner_center = [0 ,0] # [cx, cy]

        self.publisher_emo_ = self.create_publisher(String, "emo", 10)
        self.RPYpublisher = self.create_publisher(Vector3, "neck_rpy", 10)
        self.Zpublisher = self.create_publisher(UInt16, "neck_z", 10)

        self.subscriber_owner_exists = self.create_subscription(Bool, "owner_exists", self.callback_owner_exists, 10)
        self.subscriber_owner_center = self.create_subscription(Int16MultiArray, "owner_center", self.callback_owner_center, 10)
        self.subscriber_owner_size = self.create_subscription(Int16MultiArray, "owner_size", self.callback_owner_size, 10)

        # self.timer1_ = self.create_timer(1, self.publish_emo)
        # self.timer2_ = self.create_timer(1, self.publish_values)
        self.get_logger().info("fake_cmd Publisher has been started.")

    def publish_emo(self):
        msg = String()
        msg.data = str(self.emo)
        self.publisher_emo_.publish(msg)
        self.get_logger().info("[PUB] /emo: {}".format(msg.data))

        self.emo += 1
        if self.emo == 5:
            self.emo = 0

    def publish_values(self, r, p, y, z):
            msg = Vector3()
            msg.x = r # Replace with your actual values
            msg.y = p
            msg.z = y
            zmsg = UInt16()
            zmsg.data = z

            self.RPYpublisher.publish(msg)
            self.Zpublisher.publish(zmsg)
            self.get_logger().info('[PUB] /neck_rpy: {} {} {}'.format(msg.x, msg.y, msg.z))
            self.get_logger().info('[PUB] /neck_z: {}'.format(zmsg.data))

    def callback_owner_exists(self, sub_msg):
        self.get_logger().info("[SUB] /owner_exists: {}".format(sub_msg.data))
        self.owner_exists = sub_msg.data

    def callback_owner_center(self, sub_msg):
        self.get_logger().info("[SUB] /owner_center: {}".format(sub_msg.data))
        self.owner_center = sub_msg.data

        if self.owner_exists:
            y_cmd = self.owner_center[0] - 360
            self.publish_values(0, 0, y_cmd, 80)
        else:
            self.publish_values(0, 0, 0, 80)

    def callback_owner_size(self, sub_msg):
        self.get_logger().info("[SUB] /owner_size: {}".format(sub_msg.data))
        self.owner_size = sub_msg.data

def main(args=None):
    rclpy.init(args=args)
    node = FakeCmdNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
