import rclpy
from std_msgs.msg import Int16MultiArray
import numpy as np
from rclpy.node import Node
from ..mjbot_control.scripts.neck_controller import NeckControllerPublisher


class PersonTracking(Node):
    def __init__(self):
        # Replace 'owner_center_subscriber' with your node's name
        super().__init__('owner_center_subscriber')
        self.subscriber_owner_center = self.create_subscription(
            Int16MultiArray,
            'owner_center',
            self.owner_center_callback,
            10
        )

        # Initialize PID control parameters
        self.GoalAngles = np.array([-90, 0], dtype=float)
        self.time_interval = 0.25
        self.Kp_y = 0.02
        self.Ki_y = 0.02
        self.Kd_y = 0
        self.Kp_x = 0.02
        self.Ki_x = 0.02
        self.Kd_x = 0
        self.integral_y_error = 0
        self.angle_y = 0
        self.integral_x_error = 0
        self.angle_x = 0
        self.last_error_y = 0
        self.last_error_x = 0

    def owner_center_callback(self, msg):
        # Process the received message here
        owner_center_data = msg.data
        self.get_logger().info("SUB: /owner_center: {}".format(owner_center_data))

        # Get the current values of c_x and c_y from your tracking system
        # Implement this function as needed
        c_x, c_y = owner_center_data[0], owner_center_data[1]

        # Calculate errors and update the PID controllers
        error_y = 360 / 2 - c_y
        self.integral_y_error += (error_y +
                                  self.last_error_y) * self.time_interval / 2
        self.angle_y = (self.Kp_y * error_y +
                        self.Ki_y * self.integral_y_error +
                        self.Kd_y * (error_y - self.last_error_y) / self.time_interval)

        error_x = 640 / 2 - c_x
        self.integral_x_error += (error_x +
                                  self.last_error_x) * self.time_interval / 2
        self.angle_x = (self.Kp_x * error_x +
                        self.Ki_x * self.integral_x_error +
                        self.Kd_x * (error_x - self.last_error_x) / self.time_interval)

        print(f"error_y: {error_y}, error_x: {error_x}")

        # Update GoalAngles
        self.GoalAngles[0] = self.GoalAngles[0] + self.angle_y
        self.GoalAngles[1] = self.GoalAngles[1] + self.angle_x

        self.last_error_y = error_y
        self.last_error_x = error_x

        # Assuming you have a method named 'publish_values' in NeckControllerPublisher
        NeckPublsher = NeckControllerPublisher()
        # Adjust x, y, z, and w values
        NeckPublsher.publish_values(
            self.GoalAngles[0], self.GoalAngles[1], 0.3, 0)


def main(args=None):
    rclpy.init(args=args)
    your_node = PersonTracking()
    rclpy.spin(your_node)
    your_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
