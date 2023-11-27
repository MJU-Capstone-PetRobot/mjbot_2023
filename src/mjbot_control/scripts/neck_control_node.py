#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16, Int16MultiArray, String, Bool
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
        # Initialize with empty lists
        self.trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

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
        self.currentRPYZ = Quaternion(x=0.0, y=0.0, z=0.0, w=70.0)
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
                current_step = int((elapsed_sec / self.duration)
                                   * len(self.trajectory_steps['x']))
                msg = Quaternion(
                    x=self.trajectory_steps['x'][current_step],
                    y=self.trajectory_steps['y'][current_step],
                    # Add the z_offset here
                    z=self.trajectory_steps['z'][current_step] + self.z_offset,
                    w=self.trajectory_steps['w'][current_step]
                )
                self.RPYZpublisher.publish(msg)
            else:
                # Resetting after the last step
                self.trajectory_steps = None
                self.start_time = None


class CommandNeck(Node):
    EMOTION_FUNCTIONS = {
        # "daily": "daily",
        "wink": "tilt",
        "sad": "nod",
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
        self.last_position = Quaternion(x=0.0, y=0.0, z=0.0, w=70.0)
        self.yaw_errors = [0] * 5  # Last 5 yaw errors
        self.pitch_errors = [0] * 5  # Last 5 pitch errors
        self.current_state = self.STATE_DAILY
        self.publish_arm_mode = self.create_publisher(String, 'arm_mode', 10)
        self.touch_count = 0
        self.current_mode = "idle"

    def publish_arm_motions(self, Arm_motions):
        '''
        모터 제어
        '''
        msg = String()
        msg.data = str(Arm_motions)
        self.publish_arm_mode.publish(msg)
        self.get_logger().info('Published: %s' % msg.data)

    def setup_subscriptions(self):
        self.subscriber_emo = self.create_subscription(
            String, "emo", self.callback_emo, 10)
        self.subscription = self.create_subscription(
            Bool, 'touch', self.subscribe_callback_touch, 10)
        self.subscription_mode = self.create_subscription(
            String, 'mode', self.subscribe_callback_mode, 10)
        
    def subscribe_callback_mode(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f"Mode switched to: {self.current_mode}")
        if self.current_mode == None:
            self.current_mode = "idle"

    def subscribe_callback_touch(self, msg):
        if self.current_mode != "idle":
            return

        self.get_logger().info(f"currnet mode: {self.current_mode}")
        
        if msg.data == True:
            self.touch_count += 1
            self.get_logger().info(f'Touch count: {self.touch_count}') 
            if self.touch_count == 5:
                self.angry()
                self.publish_arm_motions("peng")
            elif self.touch_count == 10:
                self.nod()
                self.publish_arm_motions("cute")
                self.touch_count = 0




    def callback_emo(self, msg):
        self.current_state = self.STATE_EMOTION
        emotion_function = self.EMOTION_FUNCTIONS.get(msg.data)
        if emotion_function:
            getattr(self, emotion_function)()
        self.current_state = self.STATE_DAILY
        # Add a logging statement
        self.get_logger().info(f'Received emo message: {msg.data}')

    # def daily(self):
        
        

    def tilt(self):
        total_duration = 2
        num_nods = random.randint(1, 3)
        duration_per_nod = total_duration / num_nods
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_nods):
            # Tilt to the left (fixed position)
            left_trajectory, _ = self.neck_controller_publisher.interpolate(
                2.0, 0.0, 0.0, 70.0, duration_per_nod / 4)
            for k in trajectory_steps:
                trajectory_steps[k].extend(left_trajectory[k])

            # Tilt to the right (fixed position)
            right_trajectory, _ = self.neck_controller_publisher.interpolate(
                -2.0, 0.0, 0.0, 70.0, duration_per_nod / 4)
            for k in trajectory_steps:
                trajectory_steps[k].extend(right_trajectory[k])

            # Return to the center (fixed position)
            center_trajectory, start_time = self.neck_controller_publisher.interpolate(
                0.0, 0.0, 0.0, 70.0, duration_per_nod / 2)
            for k in trajectory_steps:
                trajectory_steps[k].extend(center_trajectory[k])

        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = total_duration

    def listening(self):
        total_duration = 3
        num_nods = random.randint(0, 4)
        if num_nods == 0:
            return

        duration_per_nod = total_duration / num_nods
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_nods):
            # Slight move down (fixed position)
            down_trajectory, _ = self.neck_controller_publisher.interpolate(
                0.0, -1.2, 0.0, 70.0, duration_per_nod / 4)
            for k in trajectory_steps:
                trajectory_steps[k].extend(down_trajectory[k])

            # Slight move up (fixed position)
            up_trajectory, _ = self.neck_controller_publisher.interpolate(
                0.0, 1.2, 0.0, 70.0, duration_per_nod / 4)
            for k in trajectory_steps:
                trajectory_steps[k].extend(up_trajectory[k])

            # Return to the center (fixed position)
            center_trajectory, start_time = self.neck_controller_publisher.interpolate(
                0.0, 0.0, 0.0, 70.0, duration_per_nod / 2)
            for k in trajectory_steps:
                trajectory_steps[k].extend(center_trajectory[k])

        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = total_duration

    def nod(self, duration=3):
        num_nods = random.randint(1, 3)
        duration_per_nod = duration / (3 * num_nods)
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_nods):
            # Move down (fixed position)
            down_trajectory, _ = self.neck_controller_publisher.interpolate(
                0.0, 0.0, -1, 70.0, duration_per_nod)
            for k in trajectory_steps:
                trajectory_steps[k].extend(down_trajectory[k])

            # Move up (fixed position)
            up_trajectory, _ = self.neck_controller_publisher.interpolate(
                0.0, 0.0, 1, 70.0, duration_per_nod)
            for k in trajectory_steps:
                trajectory_steps[k].extend(up_trajectory[k])

            # Return to the center (fixed position)
            center_trajectory, start_time = self.neck_controller_publisher.interpolate(
                0.0, 0.0, 0.0, 70.0, duration_per_nod)
            for k in trajectory_steps:
                trajectory_steps[k].extend(center_trajectory[k])

        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = duration

    def moving(self, duration=1):
        direction = random.choice(['left', 'right'])
        value = 2 if direction == 'left' else -2
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        # Tilt in the chosen direction (fixed position)
        tilt_trajectory, _ = self.neck_controller_publisher.interpolate(
            value, 0.0, 0.0, 70.0, duration / 2)
        for k in trajectory_steps:
            trajectory_steps[k].extend(tilt_trajectory[k])

        # Return to the center (fixed position)
        return_trajectory, start_time = self.neck_controller_publisher.interpolate(
            0.0, 0.0, 0.0, 70.0, duration / 2)
        for k in trajectory_steps:
            trajectory_steps[k].extend(return_trajectory[k])

        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = duration

    def angry(self, duration=2):
        num_shakes = 2
        duration_per_shake = duration / (3 * num_shakes)
        trajectory_steps = {'x': [], 'y': [], 'z': [], 'w': []}

        for _ in range(num_shakes):
            # Tilt up (fixed position)
            tilt_up_trajectory, _ = self.neck_controller_publisher.interpolate(
                0.0, 0.0, 0.0, 90.0, duration_per_shake)
            for k in trajectory_steps:
                trajectory_steps[k].extend(tilt_up_trajectory[k])

            # Tilt down (fixed position)
            tilt_down_trajectory, _ = self.neck_controller_publisher.interpolate(
                0.0, 0.0, 0.0, 60.0, duration_per_shake)
            for k in trajectory_steps:
                trajectory_steps[k].extend(tilt_down_trajectory[k])

            # Return to the center (fixed position)
            return_trajectory, start_time = self.neck_controller_publisher.interpolate(
                0.0, 0.0, 0.0, 70.0, duration_per_shake)
            for k in trajectory_steps:
                trajectory_steps[k].extend(return_trajectory[k])

        self.neck_controller_publisher.trajectory_steps = trajectory_steps
        self.neck_controller_publisher.start_time = start_time
        self.neck_controller_publisher.duration = duration


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
