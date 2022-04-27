import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage


class RobotRand(Node):

    def __init__(self):
        super().__init__('robot_rand')

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.entry_point = np.empty(shape=[2,0])    # Initial needle tip pose
        self.stage = np.empty(shape=[2,0])          # Current stage pose
        self.cmd = np.zeros((2,1))                  # Control output to the robot stage
        self.robot_ready = True                     # Robot free to new command
        self.entry_depth = 0.0
        self.depth = 0.0
   
    # Save entry point (only once)
    def entry_callback(self, msg):
        if (self.entry_point.size == 0):
            entry_point = msg.pose
            self.entry_point = np.array([[entry_point.position.x, entry_point.position.z]]).T
            self.entry_depth = entry_point.position.y
            self.get_logger().info('Initial Depth: y=%f' % (entry_point.position.y))

    # Get current base pose
    def robot_callback(self, msg_robot):
        # Save base pose only after getting entry point
        if (self.entry_point.size != 0):
            # Get pose from PoseStamped
            robot = msg_robot.pose
            # Get robot position
            self.stage = np.array([[robot.position.x, robot.position.z]]).T
            self.depth = robot.position.y
            # Check if max depth reached
            if (self.depth >= (self.entry_depth+90)):
                self.get_logger().info('Depth: y=%f' % (self.depth))
                self.robot_ready = False
            # Check if robot reached its goal position
            elif (np.linalg.norm(self.stage - self.cmd) <= 0.25):
                self.robot_ready = True 
                self.get_logger().info('Reached control target')

        # Send control signal only if robot is ready and after getting entry point (SPACE was hit by user)
        if (self.robot_ready == True) and (self.entry_point.size != 0):

            new_rand = np.random.uniform(-2.5, 2.5, (2,1))
            # new_rand[1] = min(new_rand[1], 1.0)
            # new_rand[1] = max(new_rand[1],-1.0)

            self.cmd = self.entry_point + new_rand

            # Send command to stage
            self.send_cmd(float(self.cmd[0]), float(self.cmd[1]))
            self.robot_ready = False

            self.get_logger().info('Stage: x=%f, z=%f' % (self.stage[0,0], self.stage[1,0]))
            self.get_logger().info('Control: x=%f, z=%f' % (self.cmd[0], self.cmd[1]))

            # Publish control output
            msg = PointStamped()
            msg.point.x = float(self.cmd[0])
            msg.point.z = float(self.cmd[1])
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_control.publish(msg)

    # Send MoveStage action to Stage node (Goal)
    def send_cmd(self, x, z):
        goal_msg = MoveStage.Goal()
        goal_msg.x = x
        goal_msg.z = z
        goal_msg.eps = 0.0

        # self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()  
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        # self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Get MoveStage action progress messages (Feedback)
    # def feedback_callback(self, feedback):
        # self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.x))

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        # self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        # if status == GoalStatus.STATUS_SUCCEEDED:
            # self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x))
            # self.robot_ready = True

def main(args=None):
    rclpy.init(args=args)

    robot_rand = RobotRand()

    rclpy.spin(robot_rand)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_rand.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
