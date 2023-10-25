import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import time


class NeckControllerPublisher(Node):

    def __init__(self):
        super().__init__('neck_controller_publisher')
        self.RPYpublisher = self.create_publisher(Vector3, 'float_values', 10)
        self.Zpublisher = self.create_publisher(UInt16, 'z_value', 10)
        self.dt = 0.5  # seconds
        self.rostimer = 0.05
        self.timer = self.create_timer(self.rostimer, self.publish_values)
        self.originRPY = Vector3()
        self.originZ = UInt16()
        self.originRPY.x = 0.0
        self.originRPY.y = 0.0
        self.originRPY.z = 0.0
        self.originZ.data = 70
        self.t0 = time.time()
        self.targetRPY = Vector3()
        self.targetZ = UInt16()

    def generate_trajectory(self, t0, tf, p0, pf):
        p0 = float(p0)
        pf = float(pf)
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
            0, self.dt, self.originRPY.x, r)
        a0x, a1x, a2x, a3x = self.generate_trajectory(
            0, self.dt, self.originRPY.y, p)
        a0y, a1y, a2y, a3y = self.generate_trajectory(
            0, self.dt, self.originRPY.z, y)
        a0z, a1z, a2z, a3z = self.generate_trajectory(
            0, self.dt, self.originZ.data, z)
        self.targetRPY.x = a0t + a1t * ((time.time()-self.t0)) + a2t * (
            (time.time()-self.t0)) ** 2 + a3t * ((time.time()-self.t0)) ** 3
        self.targetRPY.y = a0x + a1x * ((time.time()-self.t0)) + a2x * (
            (time.time()-self.t0)) ** 2 + a3x * ((time.time()-self.t0)) ** 3
        self.targetRPY.z = a0y + a1y * ((time.time()-self.t0)) + a2y * (
            (time.time()-self.t0)) ** 2 + a3y * ((time.time()-self.t0)) ** 3

        self.targetZ.data = round(a0z + a1z * ((time.time()-self.t0)) + a2z * (
            (time.time()-self.t0)) ** 2 + a3z * ((time.time()-self.t0)) ** 3)
        self.get_logger().info('self.targetRPY.x: %f' % self.targetRPY.x)
        self.get_logger().info('self.targetRPY.y: %f' % self.targetRPY.y)
        self.get_logger().info('self.targetRPY.z: %f' % self.targetRPY.z)
        self.get_logger().info('self.targetZ.data: %f' % self.targetZ.data)

        return time.time() - self.t0

    def publish_values(self, r, p, y, z):
        self.t0 = time.time()
        while time.time() - self.t0 < self.dt:
            self.interpolate(r, p, y, z)
            msg = Vector3()
            msg.x = self.targetRPY.x
            msg.y = self.targetRPY.y
            msg.z = self.targetRPY.z
            zmsg = UInt16()
            zmsg.data = round(self.targetZ.data)
            self.Zpublisher.publish(zmsg)
            self.RPYpublisher.publish(msg)
            time.sleep(self.rostimer)

        self.get_logger().info('self.t0: %f' % self.t0)


class AlertSubscriber(Node):
    def __init__(self):
        super().__init__('alert_subscriber')
        self.fall_down_subscription = self.create_subscription(
            Bool, 'fall_down', self.fall_down_callback, 10)
        self.co_ppm_subscription = self.create_subscription(
            Int32, 'co_ppm', self.co_ppm_callback, 10)
        self.neck_controller_publisher = NeckControllerPublisher()

    def fall_down_callback(self, msg):
        if msg.data:
            self.get_logger().info('Fall Down Alert')
            self.neck_controller_publisher.publish_values(1, 1, 1, 100)
            time.sleep(1)
            self.neck_controller_publisher.publish_values(1, 1, 1, 60)

    def co_ppm_callback(self, msg):
        if msg.data >= 200:
            self.get_logger().info('High CO PPM Alert')
            self.neck_controller_publisher.publish_values(1, 1, 1, 100)
            time.sleep(1)
            self.neck_controller_publisher.publish_values(1, 1, 1, 60)


def main(args=None):
    rclpy.init(args=args)
    alert_subscriber = AlertSubscriber()
    rclpy.spin(alert_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
