import rclpy
import numpy as np
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage
from scipy.optimize import minimize


class MPCController(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        #Declare node parameters
        self.declare_parameter('P', 10) #Prediction Horizon
        self.declare_parameter('C', 3)  #Control Horizon

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip_filtered', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning
        self.subscription_robot = self.create_subscription(PoseStamped, '/sensor/base_filtered', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from estimator node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.jacobian_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.J = np.zeros(shape=[7,2])              # Simplified Jacobian matrix

        self.tip = np.empty(shape=[7,0])            # Current needle tip pose
        self.stage = np.empty(shape=[2,0])          # Current stage pose
        now = self.get_clock().now().to_msg()
        self.curr_time = now                        # Current time stamp (from robot pose message)
        self.prev_time = now                        # Previous time stamp (from robot pose message)

        self.entry_point = np.empty(shape=[7,0])    # Initial needle tip pose
        self.cmd = np.zeros((2,1))                  # Control output to the robot stage
        self.robot_ready = True                     # Robot free to new command


    # Get current base pose
    def robot_callback(self, msg_robot):
        # Save base pose only after getting entry point
        if (self.entry_point.size) != 0:
            # Get pose from PoseStamped
            robot = msg_robot.pose
            # Get robot position and add the initial entry point (home position)
            self.stage = np.array([[robot.position.x + self.entry_point[0,0], robot.position.z + self.entry_point[2,0]]]).T
            self.prev_time = self.curr_time                        # Previous time stamp (from robot pose message)
            self.curr_time = msg_robot.header.stamp            # Current time stamp (from robot pose message)

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
        self.J = np.array([J[:,0],J[:,2]]).T
       
        # Send control signal only if robot is ready and after first readings (entry point and current needle tip)
        if (self.robot_ready == True) and (self.entry_point.size != 0) and (self.tip.size != 0):
            target = np.array([[self.entry_point[0,0], self.tip[1,0], self.entry_point[2,0], \
                                self.tip[3,0], self.tip[4,0], self.tip[5,0], self.tip[6,0]]]).T

            # MPC Initialization
            u_hat = np.zeros((2,C))                                             # Initial control guess (vector with C size)

            ################################################################
            ## TODO: Calculate or receive delta_t for each control step   ##
            ################################################################
            delta_t = (self.curr_time.nanosec - self.prev_time.nanosec)*1e9     # Temporary delta_t

########################################################################
            ## MPC Functions
########################################################################
            # Define process model
            def process_model(y0, u0, u, delta_t, J):
                y = np.zeros((7,1))
                delta_u = (u-u0)/delta_t
                y = y0 + np.matmul(J,delta_u)
                return y

            # Define objective function
            def objective(u_hat):

                # Reshape u_hat (minimize flattens it)
                u_hat = np.reshape(u_hat,(2,C))
                u_hat = np.hstack((u_hat, np.repeat(u_hat[:,[C-1]], P-C, axis=1)))
                y_hat = np.zeros((7,P))

                # Initialize prediction
                y_hat0 = self.tip
                u_hat0 = self.stage

                # Simulate prediction horizon
                for k in range(0,P):
                    yp = process_model(y_hat0, u_hat0, u_hat[:,[k]], delta_t, self.J)
                    # Save predicted variable
                    y_hat[:,[k]] = yp
                    # Update initial condition for next prediction step
                    y_hat0 = y_hat[:,[k]]
                    u_hat0 = u_hat[:,[k]]

                # Cost function calculation
                wu = 0.8
                obj = np.linalg.norm(target-y_hat) + wu*np.linalg.norm(np.diff(u_hat))
                return obj 
########################################################################

            # Initial objective
            self.get_logger().info('Initial SSE Objective: %f' % (objective(u_hat)))  # calculate cost function with initial guess

            # MPC calculation
            start_time = time.time()
            solution = minimize(objective,u_hat,method='SLSQP', bounds=[(-10, 10) for i in range(0,2*C)]) # optimizes the objective function
            u = np.reshape(solution.x,(2,C))           # reshape solution (minimize flattens it)
            end_time = time.time()
            
            cost = objective(u)
            self.get_logger().info('Final SSE Objective: %f' % (objective(u))) # calculate cost function with optimization result
            self.get_logger().info('Elapsed time: %f' % (end_time-start_time))
                
            # Update controller output
            self.cmd = u

            # Send command to stage
            # Subtract the entry point because robot considers initial position to be (0,0)
            self.send_cmd(float(self.cmd[0,0])-self.entry_point[0,0], float(self.cmd[1,0])-self.entry_point[2,0])
            self.robot_ready = False

            self.get_logger().info('Control: x=%f, z=%f - Tip: x=%f, y= %f, z=%f - Target: x=%f, y=%f, z=%f' % (self.cmd[0,0], self.cmd[1,0], \
            self.tip[0,0], self.tip[1,0], self.tip[2,0], target[0,0], target[1,0], target[2,0]))    

            # Publish control output
            msg = PointStamped()
            msg.point.x = float(self.cmd[0,0]) - self.entry_point[0,0]
            msg.point.z = float(self.cmd[1,0]) - self.entry_point[2,0]
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
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x))
            self.robot_ready = True

def main(args=None):
    rclpy.init(args=args)

    mpc_controller = MPCController()

    global P
    global C
    P = mpc_controller.get_parameter('P').get_parameter_value().integer_value
    C = mpc_controller.get_parameter('C').get_parameter_value().integer_value

    rclpy.spin(mpc_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
