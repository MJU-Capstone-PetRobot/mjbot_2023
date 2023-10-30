#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16, Int16MultiArray, String
from rclpy.node import Node
import time
import random


def clamp(value, min_value, max_value):
    """Clamp the value between min_value and max_value."""
    return float(max(min_value, min(max_value, value)))


class NeckControllerPublisher(Node):
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 320

    def __init__(self):
        super().__init__('neck_controller_publisher')
        self.setup_publishers()
        self.initialize_properties()

    def setup_publishers(self):
        self.RPZpublisher = self.create_publisher(Vector3, 'neck_rpz', 10)
        self.Yawpublisher = self.create_publisher(UInt16, 'neck_z', 10)

    def initialize_properties(self):
        self.dt = 0.5  # seconds
        self.rostimer = 0.1 # 20 Hz
        self.timer = self.create_timer(self.rostimer, self.publish_values)
        self.originRPY = Vector3(x=0.0, y=0.0, z=0.0)
        self.originZ = UInt16(data=70)
        self.t0 = time.time()
        self.targetRPY = Vector3()
        self.targetZ = UInt16()

    def generate_trajectory(self, t0, tf, p0, pf):
        p0, pf = float(p0), float(pf)
        tf_t0_3 = (tf - t0) ** 3

        a0 = (pf * t0**2 * (3 * tf - t0) + p0 *
              tf**2 * (tf - 3 * t0)) / tf_t0_3
        a1 = (6 * t0 * tf * (p0 - pf)) / tf_t0_3
        a2 = (3 * (t0 + tf) * (pf - p0)) / tf_t0_3
        a3 = (2 * (p0 - pf)) / tf_t0_3

        return a0, a1, a2, a3

    def interpolate_value(self, start, end):
        a0, a1, a2, a3 = self.generate_trajectory(0, self.dt, start, end)
        t = time.time() - self.t0
        return a0 + a1 * t + a2 * t**2 + a3 * t**3

    def interpolate(self, r, p, y, z, duration):
        self.dt = duration
        self.targetRPY.x = self.interpolate_value(self.originRPY.x, r)
        self.targetRPY.y = self.interpolate_value(self.originRPY.y, p)
        self.targetRPY.z = self.interpolate_value(self.originRPY.z, y)
        self.targetZ.data = round(self.interpolate_value(self.originZ.data, z))

    def publish_values(self, r, p, y, z, duration=None):
        if duration:
            self.dt = duration  # Update the duration if provided
        self.t0 = time.time()
        while time.time() - self.t0 < self.dt:
            self.interpolate(r, p, y, z, self.dt)

            clamped_x = round(clamp(self.targetRPY.x, -5, 5)*3.0,3)
            clamped_y = round(clamp(self.targetRPY.y, -5, 5)*3.0,3)
            clamped_z = round(clamp(self.targetRPY.z, -5, 5) * 10.0,3)
            clamped_data = clamp(self.targetZ.data, 60, 100)

            msg = Vector3(x=clamped_x, y=clamped_y, z=clamped_z)
            zmsg = UInt16(data=round(clamped_data))

            self.Zpublisher.publish(zmsg)
            time.sleep(self.rostimer)
            self.RPYpublisher.publish(msg)
            


class CommandNeck(Node):
    EMOTION_FUNCTIONS = {
        "daily": "daily",
        "wink": "tilt",
        "sad": "sad",
        "angry": "angry",
        "moving": "moving",
        "mic_waiting": "listening"
    }
    STATE_DAILY = "daily"
    STATE_EMOTION = "emotion"

    def __init__(self):
        super().__init__('command_neck_node')
        self.setup_publishers_and_subscriptions()
        self.emotion = UInt16(data=0)
        self.neck_controller_publisher = NeckControllerPublisher()
        self.last_position = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_z = UInt16(data=70)
        self.yaw_errors = [0] * 5  # Last 5 yaw errors
        self.pitch_errors = [0] * 5  # Last 5 pitch errors
        self.current_state = self.STATE_DAILY

    def setup_publishers_and_subscriptions(self):
        self.emotion_publisher = self.create_publisher(UInt16, 'emotion', 10)
        self.subscriber_emo = self.create_subscription(
            String, "emo", self.callback_emo, 10)
        self.owner_center_subscription = self.create_subscription(
            Int16MultiArray, 'owner_xyz', self.owner_center_callback, 10)

    def callback_emo(self, msg):
        self.current_state = self.STATE_EMOTION
        emotion_function = self.EMOTION_FUNCTIONS.get(msg.data)
        if emotion_function:
            getattr(self, emotion_function)()
        self.current_state = self.STATE_DAILY

    def daily(self):
        self.owner_center_callback()

    def tilt(self):
        total_duration = 2  # seconds
        num_nods = random.randint(1, 3)
        duration_per_nod = total_duration / num_nods

        for _ in range(num_nods):
            # Tilt to the left from the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x + 1, self.last_position.y, self.last_position.z, self.last_z.data, duration_per_nod / 4)

            # Tilt to the right from the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x - 1, self.last_position.y, self.last_position.z, self.last_z.data, duration_per_nod / 4)

            # Return to the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_z.data, duration_per_nod / 2)

    def listening(self):
        total_duration = 3  # seconds
        num_nods = random.randint(0, 4)
        if num_nods == 0:
            return

        duration_per_nod = total_duration / num_nods

        for _ in range(num_nods):
            # Slight move down from the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y - 0.5, self.last_position.z, self.last_z.data, duration_per_nod / 4)

            # Slight move up from the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y + 0.5, self.last_position.z, self.last_z.data, duration_per_nod / 4)

            # Return to the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_z.data, duration_per_nod / 2)

    def sad(self, duration=2):
        num_nods = random.randint(1, 3)
        duration_per_nod = duration / (3 * num_nods)

        for _ in range(num_nods):
            # Move down from the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y - 1, self.last_position.z, self.last_z.data, duration_per_nod)

            # Move up
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y + 1, self.last_position.z, self.last_z.data, duration_per_nod)

            # Return to the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_z.data, duration_per_nod)

    def moving(self, duration=1):
        # Randomly choose a direction (left or right)
        direction = random.choice(['left', 'right'])
        value = 1 if direction == 'left' else -1

        # Tilt in the chosen direction from the last position
        self.neck_controller_publisher.publish_values(
            self.last_position.x + value, self.last_position.y, self.last_position.z, self.last_z.data, duration / 2)

        # Return to the last position
        self.neck_controller_publisher.publish_values(
            self.last_position.x, self.last_position.y, self.last_position.z, self.last_z.data, duration / 2)

    def angry(self, duration=2):
        num_shakes = random.randint(1, 3)
        duration_per_shake = duration / (3 * num_shakes)

        for _ in range(num_shakes):
            # Tilt up from the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_z.data + 10, duration_per_shake)

            # Tilt down from the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_z.data - 10, duration_per_shake)

            # Return to the last position
            self.neck_controller_publisher.publish_values(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_z.data, duration_per_shake)

    def owner_center_callback(self, msg=None):
        if self.current_state != self.STATE_DAILY:
            return
        if msg is None or msg.data[0] == 0:
            msg = Int16MultiArray(data=[320, 160])  # default center values

        yaw_error = (msg.data[0] - NeckControllerPublisher.IMAGE_WIDTH /
                     2) / NeckControllerPublisher.IMAGE_WIDTH
        pitch_error = (msg.data[1] - NeckControllerPublisher.IMAGE_HEIGHT /
                       2) / NeckControllerPublisher.IMAGE_HEIGHT

        # Update and calculate the moving average for yaw
        self.yaw_errors.append(yaw_error)
        self.yaw_errors.pop(0)
        avg_yaw_error = sum(self.yaw_errors) / len(self.yaw_errors)
        target_yaw = 0.1 * avg_yaw_error

        # Update and calculate the moving average for pitch
        self.pitch_errors.append(pitch_error)
        self.pitch_errors.pop(0)
        avg_pitch_error = sum(self.pitch_errors) / len(self.pitch_errors)
        target_pitch = 0.1 * avg_pitch_error

        # Directly adjust based on error without interpolation
        clamped_x = clamp(target_yaw, -5, 5)
        clamped_y = clamp(target_pitch, -5, 5)
        clamped_z = clamp(self.last_position.z, -5, 5)

        # Update last_position and last_z
        self.last_position.x = clamped_x
        self.last_position.y = clamped_y
        self.last_position.z = clamped_z
        self.last_z.data = round(clamped_z)

        msg = Vector3(x=clamped_x, y=clamped_y, z=clamped_z)
        zmsg = UInt16(data=self.last_z.data)

        self.neck_controller_publisher.Zpublisher.publish(zmsg)
        self.neck_controller_publisher.RPYpublisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandNeck()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
