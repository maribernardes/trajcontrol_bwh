import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage
from std_msgs.msg import Int8

SAFE_LIMIT = 5.0            # Maximum control output delta from entry point
CONTROL_LENGTH = 50.0       # Maximum insertion depth for control input (stops robot after that point)

class ControllerDiscrete(Node):

    def __init__(self):
        super().__init__('controller_discrete')

        #Declare node parameters
        self.declare_parameter('K', -0.5) #Controller gain

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip_filtered', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from estimator node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.jacobian_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.entry_point = np.empty(shape=[7,0])    # Initial needle tip pose
        self.tip = np.empty(shape=[7,0])            # Current needle tip pose
        self.target = np.empty(shape=[7,0])         # Current target pose
        self.stage = np.empty(shape=[2,0])          # Current stage pose
        self.cmd = np.empty((2,1))                  # Control output to the robot stage
        self.robot_idle = True                      # Robot free to new command
        self.depth = 0.0                            # Current insertion depth
        self.J = np.zeros(7,7)                      # Initial Jacobian

    # A keyboard hotkey was pressed
    def keyboard_callback(self, msg):
        # Check if max depth reached
        # Check if robot is idle 
        # Check if entry point was acquired
        # Only send new command after hitting SPACE
        if (self.depth < CONTROL_LENGTH) and (self.robot_idle == True) and (self.entry_point.size != 0) and (msg.data == 32):

            # Build Control Jacobian from estimated model Jacobian
            Jc = np.array([self.J[:,0], self.J[:,2]]).T

            # Send control signal only if robot is ready and after first readings (entry point and current needle tip)
            if (self.robot_idle == True) and (self.entry_point.size != 0) and (self.tip.size != 0)  and (self.stage.size != 0):
                # Update target (X and Z from entry point, Y and orientation from current tip)
                self.target = np.array([[self.entry_point[0,0], self.tip[1,0], self.entry_point[2,0], \
                                        self.tip[3,0], self.tip[4,0], self.tip[5,0], self.tip[6,0]]]).T     

                # Control law
                K = self.get_parameter('K').get_parameter_value().double_value  # Get K value          
                err = self.target-self.tip                                      # Control error
                self.cmd = self.stage + K*np.matmul(np.linalg.pinv(Jc),err)     # Calculate control output

                # Limit control output to SAFE_LIMIT around entry point
                self.cmd[0,0] = min(self.cmd[0,0], self.entry_point[0,0]+SAFE_LIMIT)
                self.cmd[1,0] = min(self.cmd[1,0], self.entry_point[2,0]+SAFE_LIMIT)
                self.cmd[0,0] = max(self.cmd[0,0], self.entry_point[0,0]-SAFE_LIMIT)
                self.cmd[1,0] = max(self.cmd[1,0], self.entry_point[2,0]-SAFE_LIMIT)

                # # WARNING JUST FOR TEST!!! - DELETE AFTER
                # self.cmd[0,0] = 0.0 + self.entry_point[0,0]
                # self.cmd[1,0] = 0.0 + self.entry_point[2,0]

                # Send command to stage
                self.send_cmd(float(self.cmd[0,0]), float(self.cmd[1,0]))
                self.robot_idle = False

                self.get_logger().info('Tip: x=%f, y= %f, z=%f'   % (self.tip[0,0], self.tip[1,0], self.tip[2,0]))
                self.get_logger().info('Target: x=%f, y=%f, z=%f' % (self.target[0,0], self.target[1,0], self.target[2,0]))
                self.get_logger().info('Stage: x=%f, z=%f' % (self.stage[0,0], self.stage[1,0]))
                self.get_logger().info('Control: x=%f, z=%f' % (self.cmd[0,0], self.cmd[1,0]))
                self.get_logger().info('Err: x=%f, z=%f'   % (err[0,0], err[2,0]))

                # Publish control output (data saving purposes)
                msg = PointStamped()
                msg.point = Point(x=float(self.cmd[0]), z=float(self.cmd[1]))
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_control.publish(msg)

    # Get current base pose from robot
    def robot_callback(self, msg_robot):
        # Get pose from PoseStamped
        robot = msg_robot.pose
        # Get robot position
        self.stage = np.array([[robot.position.x, robot.position.z]]).T
        # Update insertion depth (after insertion starts)
        if (self.entry_point.size != 0):
            self.depth = robot.position.y - self.entry_point.position.y

        # Check if robot reached its goal position (after cmd starts)
        if (self.robot_idle == False) and (self.cmd.size != 0):
            if (np.linalg.norm(self.stage - self.cmd) <= 0.4):
                self.robot_idle = True 

    # Get current tip pose
    def tip_callback(self, msg):
        tip = msg.pose
        self.tip = np.array([[tip.position.x, tip.position.y, tip.position.z, \
                                tip.orientation.w, tip.orientation.x, tip.orientation.y, tip.orientation.z]]).T    
    # Get current entry point
    def entry_callback(self, msg):
        # Only once
        if (self.entry_point.size == 0):
            entry_point = msg.pose
            self.entry_point = np.array([[entry_point.position.x, entry_point.position.y, entry_point.position.z, \
                                    entry_point.orientation.w, entry_point.orientation.x, entry_point.orientation.y, entry_point.orientation.z]]).T


    # Get current Jacobian matrix from Estimator node
    def jacobian_callback(self, msg):
        self.J = np.asarray(CvBridge().imgmsg_to_cv2(msg))
        # self.J = np.array([(0.9906,-0.1395,-0.5254, 0.0044, 0.0042,-0.0000, 0.0001),
        #             ( 0.0588, 1.7334,-0.1336, 0.0020, 0.0020, 0.0002,-0.0002),
        #             (-0.3769, 0.1906, 0.2970,-0.0016,-0.0015, 0.0004,-0.0004),
        #             ( 0.0000,-0.0003, 0.0017,-0.0000,-0.0000,-0.0000,-0.0000),
        #             ( 0.0004,-0.0005, 0.0015, 0.0000, 0.0000,-0.0000, 0.0000),
        #             ( 0.0058,-0.0028,-0.0015, 0.0000, 0.0000, 0.0000,-0.0000),
        #             (-0.0059, 0.0028, 0.0015,-0.0000,-0.0000, 0.0000, 0.0000)])

    # Send MoveStage action to Stage node (Goal)
    def send_cmd(self, x, z):
        goal_msg = MoveStage.Goal()
        goal_msg.x = x
        goal_msg.z = z
        goal_msg.eps = 0.0

        # self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()  
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal not successful! Result: {0}'.format(result.x))

def main(args=None):
    # Create controller_discrete node
    rclpy.init(args=args)
    controller_discrete = ControllerDiscrete()

    # Spin the controller_discrete node
    rclpy.spin(controller_discrete)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_discrete.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
