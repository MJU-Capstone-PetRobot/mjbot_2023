#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16, Int16MultiArray, String
from rclpy.node import Node
import time
import random


def clamp(value, min_value, max_value):
    """Clamp the value between min_value and max_value."""
    return float(max(min_value, min(max_value, value)))

class NeckControllerPublisher(Node):
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    def __init__(self):
        super().__init__('command_neck_node')
        self.setup_publishers_and_subscriptions()
        
        self.neck_controller_publisher = NeckControllerPublisher()
        self.last_position = Quaternion(x=0.0, y=0.0, z=0.0, w=70.0)
        self.yaw_errors = [0] * 5  # Last 5 yaw errors
        self.pitch_errors = [0] * 5  # Last 5 pitch errors
        self.current_state = self.STATE_DAILY
        self.rostimer = 0.1  # 10 Hz
    
    def setup_publishers(self):
        self.RPYZpublisher = self.create_publisher(Quaternion, 'neck_rpyz', 10)

    def initialize_properties(self):
        self.rostimer = 0.1  # 10 Hz
        self.timer = self.create_timer(self.rostimer, self.publish_values)
        self.targetRPYZ = Quaternion()
    

    def generate_trajectory(self, t0, tf, p0, pf):
        p0, pf = float(p0), float(pf)
        tf_t0_3 = (tf - t0) ** 3

        a0 = (pf * t0**2 * (3 * tf - t0) + p0 *
              tf**2 * (tf - 3 * t0)) / tf_t0_3
        a1 = (6 * t0 * tf * (p0 - pf)) / tf_t0_3
        a2 = (3 * (t0 + tf) * (pf - p0)) / tf_t0_3
        a3 = (2 * (p0 - pf)) / tf_t0_3

        return a0, a1, a2, a3
    def interpolate(self, r, p, y, z, duration):
        self.start_time = time.time()
        self.duration = duration
        self.trajectory = {
            'x': self.generate_trajectory(0, duration, self.currentRPYZ.x, r),
            'y': self.generate_trajectory(0, duration, self.currentRPYZ.y, p),
            'z': self.generate_trajectory(0, duration, self.currentRPYZ.z, y),
            'w': self.generate_trajectory(0, duration, self.currentRPYZ.w, z),
        }
        self.targetRPYZ = Quaternion(x=r, y=p, z=y, w=z)

    def publish_values(self):
        current_time = time.time()
        if self.start_time is not None and current_time - self.start_time <= self.duration:
            t = current_time - self.start_time
            # Calculate the interpolated values
            interpolated_x = sum(a * t**i for i, a in enumerate(self.trajectory['x']))
            interpolated_y = sum(a * t**i for i, a in enumerate(self.trajectory['y']))
            interpolated_z = sum(a * t**i for i, a in enumerate(self.trajectory['z']))
            interpolated_w = sum(a * t**i for i, a in enumerate(self.trajectory['w']))

            # Clamping the interpolated values
            clamped_x = clamp(interpolated_x, -5, 5)
            clamped_y = clamp(interpolated_y, -5, 5)
            clamped_z = clamp(interpolated_z, -5, 5)
            clamped_w = clamp(interpolated_w, 60.0, 100.0)

            # Publishing the clamped values
            msg = Quaternion(x=clamped_x, y=clamped_y, z=clamped_z, w=clamped_w)
            self.RPYZpublisher.publish(msg)

            # Updating currentRPYZ
            self.currentRPYZ = Quaternion(x=clamped_x, y=clamped_y, z=clamped_z, w=clamped_w)
        else:
            # Resetting after the duration
            self.trajectory = None
            self.start_time = None

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
        
        self.neck_controller_publisher = NeckControllerPublisher()
        self.last_position = Quaternion(x=0.0, y=0.0, z=0.0, w=70.0)
        self.yaw_errors = [0] * 5  # Last 5 yaw errors
        self.pitch_errors = [0] * 5  # Last 5 pitch errors
        self.current_state = self.STATE_DAILY

    def setup_publishers_and_subscriptions(self):
        
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
        self.get_logger().info(f'Received emo message: {msg.data}')  # Add a logging statement

    def daily(self):
        self.owner_center_callback()

    def tilt(self):
        total_duration = 2  # seconds
        num_nods = random.randint(1, 3)
        duration_per_nod = total_duration / num_nods

        for _ in range(num_nods):
            # Tilt to the left from the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x + 1, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod / 4)
            self.neck_controller_publisher.publish_values()

            # Tilt to the right from the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x - 1, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod / 4)
            self.neck_controller_publisher.publish_values()

            # Return to the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod / 2)
            self.neck_controller_publisher.publish_values()
    def listening(self):
        total_duration = 3  # seconds
        num_nods = random.randint(0, 4)
        if num_nods == 0:
            return

        duration_per_nod = total_duration / num_nods

        for _ in range(num_nods):
            # Slight move down from the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y - 0.5, self.last_position.z, self.last_position.w, duration_per_nod / 4)
            self.neck_controller_publisher.publish_values()

            # Slight move up from the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y + 0.5, self.last_position.z, self.last_position.w, duration_per_nod / 4)
            self.neck_controller_publisher.publish_values()

            # Return to the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod / 2)
            self.neck_controller_publisher.publish_values()

    def sad(self, duration=2):
        num_nods = random.randint(1, 3)
        duration_per_nod = duration / (3 * num_nods)

        for _ in range(num_nods):
            # Move down from the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y - 1, self.last_position.z, self.last_position.w, duration_per_nod)
            self.neck_controller_publisher.publish_values()

            # Move up
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y + 1, self.last_position.z, self.last_position.w, duration_per_nod)
            self.neck_controller_publisher.publish_values()

            # Return to the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod)
            self.neck_controller_publisher.publish_values()

    def moving(self, duration=1):
        # Randomly choose a direction (left or right)
        direction = random.choice(['left', 'right'])
        value = 1 if direction == 'left' else -1

        # Tilt in the chosen direction from the last position
        self.neck_controller_publisher.interpolate(
            self.last_position.x + value, self.last_position.y, self.last_position.z, self.last_position.w, duration / 2)
        self.neck_controller_publisher.publish_values()

        # Return to the last position
        self.neck_controller_publisher.interpolate(
            self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w, duration / 2)
        self.neck_controller_publisher.publish_values()

    def angry(self, duration=2):
        num_shakes = random.randint(1, 3)
        duration_per_shake = duration / (3 * num_shakes)

        for _ in range(num_shakes):
            # Tilt up from the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w + 10, duration_per_shake)
            self.neck_controller_publisher.publish_values()

            # Tilt down from the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w - 10, duration_per_shake)
            self.neck_controller_publisher.publish_values()

            # Return to the last position
            self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_shake)
            self.neck_controller_publisher.publish_values()
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
        clamped_w = clamp(self.last_position.w, 60, 100)

        # Update last_position
        self.last_position.x = clamped_x
        self.last_position.y = clamped_y
        self.last_position.z = clamped_z
        self.last_position.w = clamped_w

        self.neck_controller_publisher.interpolate(
            clamped_x, clamped_y, clamped_z, clamped_w, self.rostimer)
        self.neck_controller_publisher.publish_values()

def main(args=None):
    rclpy.init(args=args)
    node = CommandNeck()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
