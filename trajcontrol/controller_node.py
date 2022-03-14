import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        #Declare node parameters
        self.declare_parameter('K', 0.001) #Controller gain

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning
        self.subscription_tip = self.create_subscription(PoseStamped, '/needle/state/pose_filtered', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning

        #Topics from estimator node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.jacobian_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.entry_point = np.empty(shape=[0,7])    # Initial needle tip pose
        self.tip = np.empty(shape=[0,7])            # Current needle tip pose
        self.target = np.empty(shape=[0,7])         # Current target pose
        self.cmd = np.zeros((2,1))                  # Control output to the robot stage

    # Get current entry point
    def entry_callback(self, msg):
        entry_point = msg.pose
        self.entry_point = np.array([[entry_point.position.x, entry_point.position.y, entry_point.position.z, \
                                entry_point.orientation.w, entry_point.orientation.x, entry_point.orientation.y, entry_point.orientation.z]]).T

    # Get current tip pose
    def tip_callback(self, msg):
        tip = msg.pose
        self.tip = np.array([[tip.position.x, tip.position.y, tip.position.z, \
                                tip.orientation.w, tip.orientation.x, tip.orientation.y, tip.orientation.z]]).T

    # Get current Jacobian matrix from Estimator node
    def jacobian_callback(self, msg):
        J = np.asarray(CvBridge().imgmsg_to_cv2(msg))
        Jc = np.array([J[:,0],J[:,2]]).T
        
        # Start controller only after first readings
        if (len(self.entry_point) and len(self.tip)) != 0:
            target = np.array([[self.entry_point[0,0], self.tip[1,0], self.entry_point[2,0], \
                                self.tip[3,0], self.tip[4,0], self.tip[5,0], self.tip[6,0]]]).T

            K = self.get_parameter('K').get_parameter_value().double_value         # Get K value
            self.cmd = self.cmd + K*np.matmul(np.linalg.pinv(Jc),self.tip-target)  # Calculate control output

            # Send command to stage
            self.send_cmd(float(self.cmd[0]), float(self.cmd[1]))
            
            # Publish control output
            msg = PointStamped()
            msg.point.x = float(self.cmd[0])
            msg.point.z = float(self.cmd[1])
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_control.publish(msg)
            #self.get_logger().info('Publish - Control cmd: %s' %  self.cmd)

    # Send MoveStage action to Stage node (Goal)
    def send_cmd(self, x, z):

        goal_msg = MoveStage.Goal()
        goal_msg.x = x
        goal_msg.z = z
        goal_msg.eps = 0.0

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        
        self.get_logger().info('Action stage - Sending goal request... Control u: x=%f, z=%f' % (goal_msg.x, goal_msg.z))      
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Get MoveStage action progress messages (Feedback)
    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.x))

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
