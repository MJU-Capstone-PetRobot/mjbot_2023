#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16, Int16MultiArray, String
from rclpy.node import Node
import time
import random


def clamp(value, min_value, max_value):
    """Clamp the value between min_value and max_value."""
    return max(min_value, min(max_value, float(value)))

class NeckControllerPublisher(Node):
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480

    Z_OFFSET = 1.0
    X_MIN, X_MAX = -3.0, 3.0
    Y_MIN, Y_MAX = -3.0, 3.0
    Z_MIN, Z_MAX = -3.0, 3.0
    W_MIN, W_MAX = 60.0, 100.0

    def __init__(self):
        super().__init__('neck_controller')
        self.setup_publishers()
        self.initialize_properties()
        self.z_offset = self.Z_OFFSET  # Initialize the offset
        self.last_position = Quaternion(x=0.0, y=0.0, z=0.0, w=70.0)
        self.yaw_errors = [0] * 5  # Last 5 yaw errors
        self.pitch_errors = [0] * 5  # Last 5 pitch errors
        self.rostimer = 0.1  # 10 Hz
        self.step_index = 0
        self.trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}  # Initialize with empty lists


    def clamp_trajectory(self, trajectory, min_value, max_value):
        return [max(min_value, min(max_value, value)) for value in trajectory]

    def interpolate(self, r, p, y, z, duration, steps=5):
        start_time = self.get_clock().now()
        step_duration = duration / steps
        trajectory_steps = {
            'x': self.clamp_trajectory(self.generate_trajectory(0, duration, self.currentRPYZ.x, r, steps), self.X_MIN, self.X_MAX),
            'y': self.clamp_trajectory(self.generate_trajectory(0, duration, self.currentRPYZ.y, p, steps), self.Y_MIN, self.Y_MAX),
            'z': self.clamp_trajectory(self.generate_trajectory(0, duration, self.currentRPYZ.z, y, steps), self.Z_MIN, self.Z_MAX),
            'w': self.clamp_trajectory(self.generate_trajectory(0, duration, self.currentRPYZ.w, z, steps), self.W_MIN, self.W_MAX),
        }
        return trajectory_steps, start_time

    def setup_publishers(self):
        self.RPYZpublisher = self.create_publisher(Quaternion, 'neck_rpyz', 10)

    def initialize_properties(self):
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_values)
        self.currentRPYZ = Quaternion()
        self.targetRPYZ = Quaternion()
        self.trajectory = None
        self.start_time = self.get_clock().now()
        self.duration = 0
    def generate_trajectory(self, t0, tf, p0, pf, steps):
        step_duration = (tf - t0) / steps
        return [self.interpolate_value(p0, pf, step_duration * i) for i in range(steps + 1)]


    def interpolate_value(self, p0, pf, t):
    # Coefficients for cubic interpolation with zero velocities at start and end
        a0 = p0
        a1 = 0  # assuming initial velocity is 0
        a2 = 3 * (pf - p0)
        a3 = -2 * (pf - p0)
        return float(a0 + a1 * t + a2 * t**2 + a3 * t**3)


    def publish_values(self):
        now = self.get_clock().now()
        if self.trajectory_steps and self.start_time:
            elapsed = now - self.start_time
            elapsed_sec = elapsed.nanoseconds / 1e9
            if elapsed_sec < self.duration:
                # Calculate the current step based on elapsed time
                current_step = int((elapsed_sec / self.duration) * len(self.trajectory_steps['x']))
                msg = Quaternion(
                    x=self.trajectory_steps['x'][current_step],
                    y=self.trajectory_steps['y'][current_step],
                    z=self.trajectory_steps['z'][current_step] + self.z_offset,  # Add the z_offset here
                    w=self.trajectory_steps['w'][current_step] +10.0
                )
                self.RPYZpublisher.publish(msg)
            else:
                # Resetting after the last step
                self.trajectory_steps = None
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

    def __init__(self, neck_controller_publisher):
        super().__init__('commands_neck_node')
        self.neck_controller_publisher = neck_controller_publisher
        self.setup_subscriptions()
        self.setup_subscriptions()
        self.last_position = Quaternion(x=0.0, y=0.0, z=0.0, w=70.0)
        self.yaw_errors = [0] * 5  # Last 5 yaw errors
        self.pitch_errors = [0] * 5  # Last 5 pitch errors
        self.current_state = self.STATE_DAILY

    def setup_subscriptions(self):
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
        # Add a logging statement
        self.get_logger().info(f'Received emo message: {msg.data}')

    def daily(self):
            self.owner_center_callback()
    def tilt(self):
        total_duration = 2  # seconds
        num_nods = random.randint(1, 3)
        duration_per_nod = total_duration / num_nods
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_nods):
            # Tilt to the left
            left_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x + 2.0, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod / 4)  # Use self.last_position.w here
            for k in trajectory_steps:
                trajectory_steps[k].extend(left_trajectory[k])

            # Tilt to the right
            right_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x - 2.0, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod / 4)  # And here
            for k in trajectory_steps:
                trajectory_steps[k].extend(right_trajectory[k])

            # Return to the last position
            center_trajectory, start_time = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, self.last_position.w, duration_per_nod / 2)  # And here
            for k in trajectory_steps:
                trajectory_steps[k].extend(center_trajectory[k])

        # Set the combined trajectory in the publisher
        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = total_duration


    def listening(self):
        total_duration = 3  # seconds
        num_nods = random.randint(0, 4)
        if num_nods == 0:
            return

        duration_per_nod = total_duration / num_nods
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_nods):
            # Slight move down
            down_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y - 1.2, self.last_position.z, 70.0, duration_per_nod / 4)
            for k in trajectory_steps:
                trajectory_steps[k].extend(down_trajectory[k])

            # Slight move up
            up_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y + 1.2, self.last_position.z, 70.0, duration_per_nod / 4)
            for k in trajectory_steps:
                trajectory_steps[k].extend(up_trajectory[k])

            # Return to the last position
            center_trajectory, start_time = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, 70.0, duration_per_nod / 2)
            for k in trajectory_steps:
                trajectory_steps[k].extend(center_trajectory[k])

        # Set the combined trajectory in the publisher
        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = total_duration


    def sad(self, duration=2):
        num_nods = random.randint(1, 3)
        duration_per_nod = duration / (3 * num_nods)
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_nods):
            # Move down from the last position
            down_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y , self.last_position.z- 1, 70.0, duration_per_nod)
            for k in trajectory_steps:
                trajectory_steps[k].extend(down_trajectory[k])

            # Move up
            up_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y , self.last_position.z+ 1, 70.0, duration_per_nod)
            for k in trajectory_steps:
                trajectory_steps[k].extend(up_trajectory[k])

            # Return to the last position
            center_trajectory, start_time = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, 70.0, duration_per_nod)
            for k in trajectory_steps:
                trajectory_steps[k].extend(center_trajectory[k])

        # Set the combined trajectory in the publisher
        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = duration

    def moving(self, duration=1):
        direction = random.choice(['left', 'right'])
        value = 2 if direction == 'left' else -2
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        # Tilt in the chosen direction from the last position
        tilt_trajectory, _ = self.neck_controller_publisher.interpolate(
            self.last_position.x + value, self.last_position.y, self.last_position.z, 70.0, duration / 2)
        for k in trajectory_steps:
            trajectory_steps[k].extend(tilt_trajectory[k])

        # Return to the last position
        return_trajectory, start_time = self.neck_controller_publisher.interpolate(
            self.last_position.x, self.last_position.y, self.last_position.z, 70.0, duration / 2)
        for k in trajectory_steps:
            trajectory_steps[k].extend(return_trajectory[k])

        # Set the combined trajectory in the publisher
        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = duration


    def angry(self, duration=2):
        num_shakes = random.randint(1, 3)
        duration_per_shake = duration / (3 * num_shakes)
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_shakes):
            # Tilt up from the last position
            tilt_up_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, 70.0 + 10, duration_per_shake)
            for k in trajectory_steps:
                trajectory_steps[k].extend(tilt_up_trajectory[k])

            # Tilt down from the last position
            tilt_down_trajectory, _ = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, 70.0 - 10, duration_per_shake)
            for k in trajectory_steps:
                trajectory_steps[k].extend(tilt_down_trajectory[k])

            # Return to the last position
            return_trajectory, start_time = self.neck_controller_publisher.interpolate(
                self.last_position.x, self.last_position.y, self.last_position.z, 70.0, duration_per_shake)
            for k in trajectory_steps:
                trajectory_steps[k].extend(return_trajectory[k])

        # Set the combined trajectory in the publisher
        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = duration


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


     

        self.neck_controller_publisher.publish_values()


def main(args=None):
    rclpy.init(args=args)
    neck_controller_node = NeckControllerPublisher()
    command_neck_node = CommandNeck(neck_controller_node)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(neck_controller_node)
    executor.add_node(command_neck_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        neck_controller_node.destroy_node()
        command_neck_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
