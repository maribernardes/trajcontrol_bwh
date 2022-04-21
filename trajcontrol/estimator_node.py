import rclpy
import message_filters
import math
import numpy as np
import quaternion

from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge.core import CvBridge
from numpy import linalg
from ros2_igtl_bridge.msg import Transform

class EstimatorNode(Node):

    def __init__(self):
        super().__init__('estimator_node')

        #Declare node parameters
        self.declare_parameter('alpha', 0.65) #Jacobian update parameter

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from sensor processing node
        self.subscription_sensor = self.create_subscription(PoseStamped, '/needle/state/pose_filtered', self.sensor_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Published topics
        self.publisher_jacobian = self.create_publisher(Image, '/needle/state/jacobian', 10)
        
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Initialize Jacobian with estimated values from previous experiments
        # (Alternative: initialize with values from first two sets of sensor and robot data)
        self.J = np.array([(-0.3482, 0.1089, 0.0893,-0.1670, 0.1967, 0.0913, 0.1103),
                  ( 0.3594, 0.1332,-0.2593, 0.1975, 0.7322, 0.7989, 0.0794),
                  (-0.1714, 0.0723, 0.1597, 0.8766, 0.0610,-0.4968, 0.2415),
                  ( 0.0003, 0.0000,-0.0005, 0.0079, 0.0007,-0.0025, 0.0021),
                  (-0.0004,-0.0001, 0.0006,-0.0077,-0.0006, 0.0025,-0.0020),
                  (-0.0017,-0.0006, 0.0009, 0.0040, 0.0083, 0.0053,-0.0007),
                  (0, 0, 0, 0, 0, 0, 0)])

        self.Z = np.empty(shape=[7,0])                  # Current needle tip pose Z = [x_tip, y_tip, z_tip, q_tip] 
        self.X = np.empty(shape=[7,0])                  # Current needle base pose X = [x_robot, y_needle_depth, z_robot, q_needle_roll]
        self.Xant = np.empty(shape=[7,0])               # Previous X = [x_robot, y_needle_depth, z_robot, q_needle_roll]
        self.Zant = np.empty(shape=[7,0])               # Previous Z = [x_tip, y_tip, z_tip, q_tip] 
        self.TXant = self.get_clock().now().to_msg()    # Previous X instant (time)
        self.TZant = self.get_clock().now().to_msg()    # Previous Z instant (time)
        
    # Get current needle tip from sensor processing node
    # Z = [x_tip, y_tip, z_tip, q_tip] (Obs: for q, roll=pitch)
    def sensor_callback(self, msg_sensor):
        # Get filtered sensor in robot frame        
        self.Z = np.array([[msg_sensor.pose.position.x, msg_sensor.pose.position.y, msg_sensor.pose.position.z, \
            msg_sensor.pose.orientation.w, msg_sensor.pose.orientation.x, msg_sensor.pose.orientation.y, msg_sensor.pose.orientation.z]]).T

    # needle_pose from robot node
    # X = [x_robot, y_needle_depth, z_robot, q_needle_roll]
    # Get estimator input X
    # Perform estimator "correction" from last Z (aurora)
    def robot_callback(self, msg_robot):
        # Get pose from PoseStamped
        robot = msg_robot.pose

        # Already stored readings: update Jacobian
        if (self.Xant.size != 0) and (self.Zant.size != 0):   
            # From robot, get input X
            self.X = np.array([[robot.position.x, robot.position.y, robot.position.z, \
                robot.orientation.w, robot.orientation.x, robot.orientation.y, robot.orientation.z]]).T
            TX = msg_robot.header.stamp
            TZ = TX #For now, consider simultaneous robot and Aurora readings

            # Calculate deltaTX and deltaTZ between current and previous robot needle_pose            
            deltaTX = ((TX.sec*1e9 + TX.nanosec) - (self.TXant.sec*1e9 + self.TXant.nanosec))*1e-9    
            deltaTZ = deltaTX #For now, consider simultaneous robot and Aurora readings

            # Calculate deltaX and deltaZ between current and previous robot needle_pose 
            deltaX = (self.X - self.Xant)/deltaTX
            deltaZ = (self.Z - self.Zant)/deltaTZ

            # Update Jacobian
            alpha = self.get_parameter('alpha').get_parameter_value().double_value
            self.J = self.J + alpha*np.matmul(((deltaZ-np.matmul(self.J, deltaX))/(np.matmul(np.transpose(deltaX), deltaX)+1e-9)), np.transpose(deltaX))
            
            # Publish new Jacobian
            msg = CvBridge().cv2_to_imgmsg(self.J)
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_jacobian.publish(msg)
            # self.get_logger().info('Publish - Jacobian: %s' %  self.J)

            # Save last readings
            self.Zant = self.Z
            self.Xant = self.X
            self.TXant = TX
            self.TZant = TZ

        # First readings: initialize variables
        else:
            if (self.Z.size != 0):
                self.Zant = self.Z
                self.TZant = self.TXant #For now, consider simultaneous robot and Aurora readings
            # From robot, get input X
            self.Xant = np.array([[robot.position.x, robot.position.y, robot.position.z, \
                robot.orientation.w, robot.orientation.x, robot.orientation.y, robot.orientation.z]]).T
            self.TXant = msg_robot.header.stamp
            # From previous aurora reading, get input Z

def main(args=None):
    rclpy.init(args=args)

    estimator_node = EstimatorNode()

    rclpy.spin(estimator_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
