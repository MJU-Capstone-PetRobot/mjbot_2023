import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16
from rclpy.node import Node
from example_interfaces.msg import Bool
from example_interfaces.msg import Int32
import numpy as np
import time


class NeckControllerPublisher(Node):

    def __init__(self):
        super().__init__('float_publisher')
        self.RPYpublisher = self.create_publisher(Vector3, 'float_values', 10)
        self.Zpublisher = self.create_publisher(UInt16, 'z_value', 10)
        self.dt = 0.5  # seconds
        self.rostimer = 0.02
        self.timer = self.create_timer(self.rostimer, self.publish_values)
        self.originRPY = Vector3()
        self.originRPY = [0, 0, 0]
        self.originZ = UInt16()
        self.originZ = 70
        self.targetRPY = Vector3()
        self.targetRPY = [0, 0, 0]
        self.targetZ = UInt16()
        self.targetZ = 70
        self.t0 = time.time()

        # 3차 경로 계획

    def generate_trajectory(t0, tf, p0, pf):
        tf_t0_3 = (tf - t0)**3
        a0 = pf*(t0**2)*(3*tf-t0) + p0*(tf**2)*(tf-3*t0)
        a0 = a0 / tf_t0_3

        a1 = 6 * t0 * tf * (p0 - pf)
        a1 = a1 / tf_t0_3

        a2 = 3 * (t0 + tf) * (pf - p0)
        a2 = a2 / tf_t0_3

        a3 = 2 * (p0 - pf)
        a3 = a3 / tf_t0_3

        return a0, a1, a2, a3

    def interpolate(self, r, p, y, z):
        a0t, a1t, a2t, a3t = self.generate_trajectory(
            self.t0, self.t0+self.dt, self.originRPY.x, r)
        a0x, a1x, a2x, a3x = self.generate_trajectory(
            self.t0, self.t0+self.dt, self.originRPY.y, p)
        a0y, a1y, a2y, a3y = self.generate_trajectory(
            self.t0, self.t0+self.dt, self.originRPY.z, y)
        a0z, a1z, a2z, a3z = self.generate_trajectory(
            self.t0, self.t0+self.dt, self.originZ, z)
        self.targetRPY.x = a0t + a1t * (self.t0-time.time()) + a2t * (
            self.t0-time.time()) ** 2 + a3t * (self.t0-time.time()) ** 3
        self.targetRPY.y = a0x + a1x * (self.t0-time.time()) + a2x * (
            self.t0-time.time()) ** 2 + a3x * (self.t0-time.time()) ** 3
        self.targetRPY.z = a0y + a1y * (self.t0-time.time()) + a2y * (
            self.t0-time.time()) ** 2 + a3y * (self.t0-time.time()) ** 3
        self.targetZ = a0z + a1z * (self.t0-time.time()) + a2z * \
            (self.t0-time.time()) ** 2 + a3z * (self.t0-time.time()) ** 3

        return

    def publish_values(self, r, p, y, z):
        self.interpolate(r, p, y, z)

        self.t0 = time.time()
        msg = Vector3()
        msg.x = self.targetRPY.x
        msg.y = self.targetRPY.y
        msg.z = self.targetRPY.z
        zmsg = UInt16()
        zmsg.data = self.targetZ
        self.Zpublisher.publish(zmsg)
        self.RPYpublisher.publish(msg)
        self.get_logger().info('Published float values')


class alertSubscriber(Node):
    def __init__(self):
        super().__init__('alert_subscriber')
        self.subscription = self.create_subscription(
            Bool, 'fall_down', self.subscribe_callback, 10)
        self.subscription = self.create_subscription(
            Int32, 'co_ppm', self.subscribe_callback_fire, 10)
        self.neck_controller_publisher = NeckControllerPublisher()

    def subscribe_callback(self, msg):

        self.get_logger().info('Received: %d' % msg.data)

        if msg.data == True:

            self.neck_controller_publisher.publish_values(0, 0, 0, 100)
            self.neck_controller_publisher.publish_values(0, 0, 0, 60)
            self.get_logger().info('fall down')

    def subscribe_callback_fire(self, msg):

        self.get_logger().info('Received: %s' % msg.data)

        if msg.data >= 200:

            self.neck_controller_publisher.publish_values(0, 0, 0, 100)
            self.neck_controller_publisher.publish_values(0, 0, 0, 60)
            self.get_logger().info('fall down')


def main(args=None):
    rclpy.init(args=args)
    neck_controller_publisher = NeckControllerPublisher()
    alert_subscriber = alertSubscriber()
    rclpy.spin(neck_controller_publisher)
    rclpy.spin(alert_subscriber)
    neck_controller_publisher.destroy_node()
    alert_subscriber.destroy_node()
