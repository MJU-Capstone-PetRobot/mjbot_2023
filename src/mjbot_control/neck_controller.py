import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16
from rclpy.node import Node

class NeckControllerPublisher(Node):

    def __init__(self):
        super().__init__('float_publisher')
        self.RPYpublisher = self.create_publisher(Vector3, 'float_values', 10)
        self.Zpublisher = self.create_publisher(UInt16, 'z_value', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_values)

    def publish_values(self, r,p,y,z):
        msg = Vector3()
        msg.x = r # Replace with your actual values
        msg.y = p
        msg.z = y
        zmsg = UInt16()
        zmsg.data = z
        self.Zpublisher.publish(zmsg)
        self.RPYpublisher.publish(msg)
        self.get_logger().info('Published float values')
    
