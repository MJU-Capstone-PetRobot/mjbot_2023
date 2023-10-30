
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

# ... [rest of the imports]

class NeckControllerPublisher(Node):
    # ... [rest of the class definition]

    def __init__(self):
        super().__init__('neck_controller_publisher')
        self.setup_publishers()
        self.initialize_properties()
        
        # Initialize action client for neck yaw control
        self._yaw_action_client = ActionClient(self, FollowJointTrajectory, '/neck_yaw_controller/follow_joint_trajectory')
        self.yaw_action_state = ActionState.NONE
        self.yaw_action_done_event = threading.Event()

    def setup_publishers(self):
        # ... [rest of the setup_publishers method]
        self.Yawtrajectory_msg = FollowJointTrajectory.Goal()
        self.Yawtrajectory_msg.trajectory.joint_names = ['neck_yaw_joint']  # Assuming joint name is 'neck_yaw_joint'
        self.Yawpoint = JointTrajectoryPoint()
        
    # ... [rest of the methods]
    
    def publish_values(self, r, p, y, z, duration=None):
        # ... [rest of the method]
        
        # Instead of directly publishing yaw values, send a trajectory goal
        self.Yawpoint.positions = [y]  # Set the yaw value
        self.Yawpoint.time_from_start = Duration(seconds=self.dt).to_msg()
        self.Yawtrajectory_msg.trajectory.points = [self.Yawpoint]
        if self._yaw_action_client.action_state in [ActionState.NONE, ActionState.SUCCEEDED]:
            self._yaw_action_client.send_goal(self.Yawtrajectory_msg, self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Yaw goal accepted by the action server')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        else:
            self.get_logger().info('Yaw goal rejected by the action server')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Yaw result received: {0}'.format(result))
        self.yaw_action_state = ActionState.SUCCEEDED
        self.yaw_action_done_event.set()

# ... [rest of the script]
