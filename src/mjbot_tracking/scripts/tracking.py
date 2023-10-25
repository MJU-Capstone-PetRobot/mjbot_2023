#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray


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


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_tracker', 10)
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'owner_center',
            self.callback,
            10)
        self.base_cmd = Twist()

        self.target_distance = 800

        # PID controller for yaw control
        self.yaw_pid = PIDController(0.5, 0.1, 0.1)

        self.image_width = 640
        self.image_height = 360

    def update(self, linear_speed, angular_speed):
        self.base_cmd.linear.x = linear_speed
        self.base_cmd.angular.z = angular_speed
        self.publisher.publish(self.base_cmd)
        self.get_logger().info("Published: /cmd_vel_tracker: {}".format(self.base_cmd))

    def callback(self, msg):
        person_x = msg.data[0] * 1.0
        person_y = msg.data[1] * 1.0
        person_distance = msg.data[2] * 1.0

        offset_x = person_x - self.image_width / 2
        theta = offset_x / self.image_width

        # Use the PID controller to calculate angular_speed
        angular_speed = -self.yaw_pid.update(theta)

        # Calculate linear_speed based on the person's distance
        if person_distance <= 300:
            linear_speed = 0.0
        elif person_distance >= 1200:
            linear_speed = 1.0  # Max speed is 1.0 when person_distance is greater than or equal to 1200
        else:
            # Linear interpolation between 0.4 and 1.0 based on the range of person_distance
            linear_speed = 0.4 + (person_distance - 800) * 0.2 / 400

        # Make sure the linear_speed doesn't exceed 1.0
        linear_speed = min(1.0, linear_speed)

        self.update(linear_speed, angular_speed)


def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
