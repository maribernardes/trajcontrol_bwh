from sympy import Point3D
import rclpy
import os
import numpy as np

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from stage_control_interfaces.action import MoveStage

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion, Point
from transforms3d.euler import euler2quat
from scipy.io import loadmat

class VirtualRobot(Node):

    def __init__(self):
        super().__init__('virtual_robot')
        
        #Declare node parameters
        self.declare_parameter('dataset', 'fbg_10') #Dataset file name

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning

        #Published topics
        self.publisher_needle_pose = self.create_publisher(PoseStamped, '/stage/state/needle_pose', 10)
        timer_period = 0.8  # seconds
        self.timer = self.create_timer(timer_period, self.timer_needlepose_callback)

        #Action server
        self._action_server = ActionServer(self, MoveStage, '/move_stage', execute_callback=self.execute_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)

        #Load data from matlab file
        file_path = os.path.join('src','trajcontrol','files',self.get_parameter('dataset').get_parameter_value().string_value+ '.mat') #String with full path to file
        trial_data = loadmat(file_path, mat_dtype=True)
        
        # Stored values
        self.entry_point = np.empty(shape=[7,0])    # Initial needle tip pose
        self.needle_pose = trial_data['needle_pose'][0]
        self.time_stamp = trial_data['time_stamp'][0]
        self.i = 0

    # Get current entry point
    def entry_callback(self, msg):
        entry_point = msg.pose
        self.entry_point = np.array([[entry_point.position.x, entry_point.position.y, entry_point.position.z, \
                                entry_point.orientation.w, entry_point.orientation.x, entry_point.orientation.y, entry_point.orientation.z]]).T

    # Publish current needle_pose
    def timer_needlepose_callback(self):
        # Publish needle_pose only after entry point is acquired
        if (self.entry_point.size != 0):
            # Use Aurora timestamp
            now = self.get_clock().now().to_msg()
            decimal = np.mod(self.time_stamp[self.i],1)
            now.nanosec = int(decimal*1e9)
            now.sec = int(self.time_stamp[self.i]-decimal)
            
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = "stage"

            # Populate message with Z data from matlab file and previous control input
            Z = self.needle_pose[self.i]
            new_rand = np.random.randn(2,1)
            msg.pose.position = Point(x=self.entry_point[0,0]+0.1*new_rand[0,0], y=Z[1,0], z=self.entry_point[2,0]+0.1*new_rand[1,0])
            msg.pose.orientation = Quaternion(x=float(Z[3,0]), y=float(Z[4,0]), z=float(Z[5,0]), w=float(Z[6,0]))

            self.publisher_needle_pose.publish(msg)
            self.get_logger().info('Publish - Needle pose: %s'  % (msg.pose))
            self.i += 1

    # Destroy de action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Accept or reject a client request to begin an action
    # This server allows multiple goals in parallel
    def goal_callback(self, goal_request):
        # self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    # Accept or reject a client request to cancel an action
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # Execute a goal
    # This is a dummy action: the "goal" is to increment x from 0 to 4
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MoveStage.Feedback()
        feedback_msg.x = 0.0

        # Start executing the action
        for k in range(1, 5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return MoveStage.Result()

            # Update control input
            feedback_msg.x = float(k)

            # self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.x))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()

        # Populate result message
        result = MoveStage.Result()
        result.x = feedback_msg.x

        # self.get_logger().info('Returning result: {0}'.format(result.x))

        return result


def main(args=None):
    rclpy.init(args=args)

    virtual_robot = VirtualRobot()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(virtual_robot, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
