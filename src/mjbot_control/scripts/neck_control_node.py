import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16, Bool, Int32, Int16MultiArray, String
from rclpy.node import Node
import time
import random


class PIDController:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, error):
        derivative = error - self.last_error
        self.integral += error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output


class NeckControllerPublisher(Node):

    def __init__(self):
        super().__init__('neck_controller_publisher')

        # Publishers
        self.RPYpublisher = self.create_publisher(Vector3, 'float_values', 10)
        self.Zpublisher = self.create_publisher(UInt16, 'z_value', 10)

        # Subscriptions
        self.owner_center_subscription = self.create_subscription(
            Int16MultiArray, 'owner_center', self.owner_center_callback, 10)

        # Properties
        self.dt = 0.5  # seconds
        self.rostimer = 0.05
        self.timer = self.create_timer(self.rostimer, self.publish_values)
        self.image_width = 640
        self.originRPY = Vector3(x=0.0, y=0.0, z=0.0)
        self.originZ = UInt16(data=70)
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

    def owner_center_callback(self, msg):
        yaw_error = (msg.data[0] - self.image_width / 2) / self.image_width
        target_yaw = 1 * yaw_error
        self.publish_values(target_yaw, 1, 1, 70)

    def publish_values(self, r, p, y, z):
        self.t0 = time.time()
        while time.time() - self.t0 < self.dt:
            self.interpolate(r, p, y, z)
            msg = Vector3(x=self.targetRPY.x,
                          y=self.targetRPY.y, z=self.targetRPY.z)
            zmsg = UInt16(data=round(self.targetZ.data))
            self.Zpublisher.publish(zmsg)
            self.RPYpublisher.publish(msg)
            time.sleep(self.rostimer)


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


class EmotionExpression(Node):
    def __init__(self):
        super().__init__('emotion_expression')
        self.emotion_publisher = self.create_publisher(UInt16, 'emotion', 10)
        self.subscriber_emo = self.create_subscription(
            String, "emo", self.callback_emo, 10)
        self.emotion = UInt16(data=0)
        self.neck_controller_publisher = NeckControllerPublisher()  # create an instance of NeckControllerPublisher

    def callback_emo(self, msg):
        emotion = msg.data
        if emotion == "Daily":
            self.neck_controller_publisher.publish_values(0, 0, 0, 70)
        elif emotion == "wink":
            self.tilt(direction='left')
        elif emotion == "sad":
            self.nod()
        elif emotion == "angry":
            self.shake()
        elif emotion == "moving":
            self.turn()
        elif emotion == "mic_waiting":
            # Example: Tilt upwards (listening pose)
            self.listening()
            self.neck_controller_publisher.publish_values(0, 1, 0, 70)
    
    def listening(self):
        # """A subtle nodding  gesture to indicate that the robot is listening"""
        total_duration = 3  # seconds
        num_nods = random.randint(0, 4)  # Random number of nods between 0 and 4
        
        if num_nods == 0:
            time.sleep(total_duration)
            return

        duration_per_nod = total_duration / num_nods
        
        for _ in range(num_nods):
            # Slight move down
            self.neck_controller_publisher.publish_values(0, -0.5, 0, 70)
            time.sleep(duration_per_nod / 4)
            # Slight move up
            self.neck_controller_publisher.publish_values(0, 0.5, 0, 70)
            time.sleep(duration_per_nod / 4)
            # Return to neutral
            self.neck_controller_publisher.publish_values(0, 0, 0, 70)
            time.sleep(duration_per_nod / 2)

    def nod(self, duration=1):
        # Move down
        self.neck_controller_publisher.publish_values(0, -1, 0, 70)
        time.sleep(duration/2)
        # Move up
        self.neck_controller_publisher.publish_values(0, 1, 0, 70)
        time.sleep(duration/2)

    def tilt(self, direction='left', duration=1):
        # Tilt left or right based on the direction
        value = 1 if direction == 'left' else -1
        self.neck_controller_publisher.publish_values(value, 0, 0, 70)
        time.sleep(duration)

    def shake(self, duration=1):
        # Move to the left
        self.neck_controller_publisher.publish_values(1, 0, 0, 70)
        time.sleep(duration/3)
        # Move to the right
        self.neck_controller_publisher.publish_values(-1, 0, 0, 70)
        time.sleep(duration/3)
        # Return to the center
        self.neck_controller_publisher.publish_values(0, 0, 0, 70)
        time.sleep(duration/3)

    def turn(self, direction='left', duration=1):
        # Turn to the left or right based on the direction
        value = 1 if direction == 'left' else -1
        self.neck_controller_publisher.publish_values(value, 0, 0, 70)
        time.sleep(duration)

def main(args=None):
    rclpy.init(args=args)
    alert_subscriber = AlertSubscriber()
    rclpy.spin(alert_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
