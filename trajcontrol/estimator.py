import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge.core import CvBridge

class Estimator(Node):

    def __init__(self):
        super().__init__('estimator')

        #Declare node parameters
        self.declare_parameter('alpha', 0.65) #Jacobian update parameter

        #Topics from sensor processing node
        self.subscription_sensor = self.create_subscription(PoseStamped, '/sensor/tip_filtered', self.sensor_callback, 10)
        self.subscription_sensor # prevent unused variable warning
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Published topics
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_jacobian_callback)
        self.publisher_jacobian = self.create_publisher(Image, '/needle/state/jacobian', 10)
        
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Initialize Jacobian with estimated values from previous experiments
        # (Alternative: initialize with values from first two sets of sensor and robot data)
        self.J = np.array([(0.9906,-0.1395,-0.5254),
                    ( 0.0588, 1.7334,-0.1336),
                    (-0.3769, 0.1906, 0.2970),
                    ( 0.0004,-0.0005, 0.0015),
                    ( 0.0058,-0.0028,-0.0015)])
        self.Z = np.empty(shape=[5,0])                  # Current needle tip pose Z = [x_tip, y_tip, z_tip, yaw, pitch] 
        self.X = np.empty(shape=[3,0])                  # Current needle base pose X = [x_robot, y_needle_depth, z_robot]
        self.Xant = np.empty(shape=[3,0])               # Previous X = [x_robot, y_needle_depth, z_robot]
        self.Zant = np.empty(shape=[6,0])               # Previous Z = [x_tip, y_tip, z_tip, yaw, pitch] 
        self.TX = self.get_clock().now().to_msg()       # Current X instant (time)
        self.TZ = self.get_clock().now().to_msg()       # Current Z instant (time)        
        self.TXant = self.get_clock().now().to_msg()    # Previous X instant (time)
        self.TZant = self.get_clock().now().to_msg()    # Previous Z instant (time)
        
    # Publish Jacobian 
    def timer_jacobian_callback(self):
        # Publish new Jacobian
        msg = CvBridge().cv2_to_imgmsg(self.J)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_jacobian.publish(msg)
        # self.get_logger().info('Publish - Jacobian: %s' %  self.J)
 
    # Get current needle tip from sensor processing node
    # Z = [x_tip, y_tip, z_tip, yaw, pitch]  
    def sensor_callback(self, msg_sensor):
        # Get filtered sensor in robot frame   
        quat = np.array([[msg_sensor.pose.orientation.w, msg_sensor.pose.orientation.x, msg_sensor.pose.orientation.y, msg_sensor.pose.orientation.z]]).T
        angles = quat2ypr(quat)
        self.Zant = self.Z
        self.TZant = self.TZ
        self.Z = np.array([[msg_sensor.pose.position.x, msg_sensor.pose.position.y, msg_sensor.pose.position.z, angles[0], angles[1]]]).T
        self.TZ = msg_sensor.header.stamp

    # needle_pose from robot node
    # X = [x_robot, y_needle_depth, z_robot]
    # Get estimator input X
    # Perform estimator "correction" from last Z (aurora)
    def robot_callback(self, msg_robot):
        # Get pose from PoseStamped
        robot = msg_robot.pose
        # From robot, get input X
        self.Xant = self.X
        self.TXant = self.TX
        self.X = np.array([[robot.position.x, robot.position.y, robot.position.z]]).T
        self.TX = msg_robot.header.stamp

        # Already stored both Xant and Zant: update Jacobian
        if (self.Xant.size != 0) and (self.Zant.size != 0) :
            # Calculate deltaTX and deltaTZ between current and previous robot needle_pose            
            deltaTX = ((self.TX.sec*1e9 + self.TX.nanosec) - (self.TXant.sec*1e9 + self.TXant.nanosec))*1e-9    
            deltaTZ = ((self.TZ.sec*1e9 + self.TZ.nanosec) - (self.TZant.sec*1e9 + self.TZant.nanosec))*1e-9    

            # Calculate deltaX and deltaZ between current and previous robot needle_pose 
            deltaX = (self.X - self.Xant)/deltaTX
            deltaZ = (self.Z - self.Zant)/deltaTZ

            # Update Jacobian
            alpha = self.get_parameter('alpha').get_parameter_value().double_value
            self.J = self.J + alpha*np.outer((deltaZ-np.matmul(self.J, deltaX))/(np.matmul(np.transpose(deltaX), deltaX)+1e-9), deltaX)


########################################################################
### Auxiliar functions ###
########################################################################

# Function: quat2ypr
# DO: Transform quaternion representation to yaw-pitch-roll (Tait-Bryan Z-Y'-X'')
# Inputs: 
#   q: quaternion (numpy array [qw, qx, qy, qz])
# Output:
#   angles: angle vector (numpy array [yaw, pitch, roll])

def quat2ypr(q):

    angles = np.empty(shape=[3,0]) 

    # yaw (z-axis rotation) [-pi, pi]
    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    angles[0] = math.atan2(siny_cosp, cosy_cosp)

    # pitch (y-axis rotation) [-pi/2, pi/2]
    sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    if (math.abs(sinp) >= 1):
        angles[1] = math.copysign(math.pi / 2, sinp); # use 90 degrees if out of range
    else:
        angles[1] = math.asin(sinp)

    # roll (x-axis rotation) [-pi, pi]
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    angles[2] = math.atan2(sinr_cosp, cosr_cosp)

    
    return angles

########################################################################
def main(args=None):
    rclpy.init(args=args)

    estimator = Estimator()

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
