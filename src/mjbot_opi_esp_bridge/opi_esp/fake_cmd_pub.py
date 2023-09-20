#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16

class FakeCmdNode(Node):
    def __init__(self):
        super().__init__("fake_cmd")
        self.emo = 0

        self.publisher_emo_ = self.create_publisher(String, "emo", 10)
        self.RPYpublisher = self.create_publisher(Vector3, "neck_rpy", 10)
        self.Zpublisher = self.create_publisher(UInt16, "neck_z", 10)

        self.timer1_ = self.create_timer(1, self.publish_emo)
        self.timer2_ = self.create_timer(1, self.publish_values)
        self.get_logger().info("fake_cmd Publisher has been started.")

    def publish_emo(self):
        msg = String()
        msg.data = str(self.emo)
        self.publisher_emo_.publish(msg)
        self.get_logger().info("[PUB] /emo: {}".format(msg.data))

        self.emo += 1
        if self.emo == 5:
            self.emo = 0

    def publish_values(self):
            msg = Vector3()
            msg.x = 5.0 # Replace with your actual values
            msg.y = -5.0
            msg.z = 5.0
            zmsg = UInt16()
            zmsg.data = 80

            self.RPYpublisher.publish(msg)
            self.Zpublisher.publish(zmsg)
            self.get_logger().info('[PUB] /neck_rpy: {} {} {}'.format(msg.x, msg.y, msg.z))
            self.get_logger().info('[PUB] /neck_z: {}'.format(zmsg.data))

def main(args=None):
    rclpy.init(args=args)
    node = FakeCmdNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
