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

        #Topics from UI node
        self.subscription_UI = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_UI  # prevent unused variable warning

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

        self.Xant = np.zeros((7,1))                     # Previous X = [x_robot, y_needle_depth, z_robot, q_needle_roll]
        self.Zant = np.zeros((7,1))                     # Previous Z = [x_tip, y_tip, z_tip, q_tip] 
        self.TXant = self.get_clock().now().to_msg()    # Previous X instant (time)
        self.TZant = self.get_clock().now().to_msg()    # Previous Z instant (time)

        self.entry_point = np.empty(shape=[0,7])      # Initial needle tip pose
        self.Z = np.empty(shape=[0,7])                # Last needle tip pose
        self.i = 0                                    # Estimation step
        
    # Get current entry point from UI node
    def entry_point_callback(self, msg):
        entry_point = msg.pose
        self.entry_point = np.array([[entry_point.position.x, entry_point.position.y, entry_point.position.z, \
                                entry_point.orientation.w, entry_point.orientation.x, entry_point.orientation.y, entry_point.orientation.z]]).T

    # Get current needle tip from sensor processing node
    # Z = [x_tip, y_tip, z_tip, q_tip] (Obs: for q, roll=pitch)
    def sensor_callback(self, msg_sensor):
        # Get filtered sensor in robot frame        
        self.Z = np.array([[msg_sensor.pose.position.x, msg_sensor.pose.position.y, msg_sensor.pose.position.z, \
            msg_sensor.pose.orientation.w, msg_sensor.pose.orientation.x, msg_sensor.pose.orientation.y, msg_sensor.pose.orientation.z]]).T
        #self.get_logger().info('Z = %s in %s frame' % (self.Z.T, msg_sensor.header.frame_id))

    # Get current needle_pose from robot node
    # X = [x_robot, y_needle_depth, z_robot, q_needle_roll]
    # Get estimator input X
    # Perform estimator "correction" from last Z (aurora)
    def robot_callback(self, msg_robot):

        # Start estimator only after getting initial position (entry point)
        if len(self.entry_point) != 0:
            # Get pose from PoseStamped
            robot = msg_robot.pose
            TX = msg_robot.header.stamp
            
            # From robot, get input X and add the initial entry point (home position)
            X = np.array([[robot.position.x + self.entry_point[0,0], robot.position.y + self.entry_point[1,0], robot.position.z + self.entry_point[2,0], \
                robot.orientation.w, robot.orientation.x, robot.orientation.y, robot.orientation.z]]).T

            deltaTX = ((TX.sec*1e9 + TX.nanosec) - (self.TXant.sec*1e9 + self.TXant.nanosec))*1e-9
            
            #For now, consider simultaneous robot and Aurora readings
            TZ = TX
            deltaTZ = deltaTX

            deltaZ = (self.Z - self.Zant)/deltaTZ
            deltaX = (X - self.Xant)/deltaTX

            self.Zant = self.Z
            self.Xant = X
            self.TXant = TX
            self.TZant = TZ

            alpha = self.get_parameter('alpha').get_parameter_value().double_value
            if (self.i > 0): #Does nothing if first sample (no deltas)
                self.J = self.J + alpha*np.matmul(((deltaZ-np.matmul(self.J, deltaX))/(np.matmul(np.transpose(deltaX), deltaX)+1e-9)), np.transpose(deltaX))
            self.i += 1
            # self.get_logger().info('Sample #%i: X = %s in %s frame' % (self.i, X.T, msg_robot.header.frame_id))
            
            # Publish new Jacobian
            msg = CvBridge().cv2_to_imgmsg(self.J)
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_jacobian.publish(msg)
            # self.get_logger().info('Publish - Jacobian: %s' %  self.J)

########################################################################
### Auxiliar functions ###

# Function: upforw2quat
# DO:Get quaternion from up and forward vectors
# Input: up and forward vectors (3d float arrays)
# Output: quaternion (4d float array)
def upforw2quat(up, forw):
    left = np.cross(up, forw)
    up = np.cross(forw, left) # make up orthogonal

    # normalize vector just in case
    up = up/np.linalg.norm(up)
    forw = forw/np.linalg.norm(forw)
    left = left/np.linalg.norm(left)

    # build rotation matrix M
    m11 = left[0]
    m12 = left[1]
    m13 = left[2]
    m21 = up[0]
    m22 = up[1]
    m23 = up[2]
    m31 = forw[0]
    m32 = forw[1]
    m33 = forw[2]

    tr = m11+m22+m33 # trace of M
    if (tr>0):
        s = 2*math.sqrt(tr+1.0)
        q0 = 0.25*s
        q1 = (m32-m23)/s
        q2 = (m13-m31)/s
        q3 = (m21-m12)/s
    else:
        if ((m11>m22) and (m11>m33)):
            s = 2*math.sqrt(1.0+m11-m22-m33)
            q0 = (m32-m23)/s
            q1 = 0.25*s
            q2 = (m12+m21)/s
            q3 = (m13+m31)/s
        else:
            if (m22>m33):
                s = 2*math.sqrt(1.0+m22-m11-m33)
                q0 = (m13-m31)/s
                q1 = (m12+m21)/s
                q2 = 0.25*s
                q3 = (m23+m32)/s
            else:
                s = 2*math.sqrt(1.0+m33-m11-m22)
                q0 = (m21-m12)/s
                q1 = (m13+m31)/s
                q2 = (m23+m32)/s
                q3 = 0.25*s
    q = [q0, q1, q2, q3]
    return q/np.linalg.norm(q) # normalize q to return unit quaternion

########################################################################

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
