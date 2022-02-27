import os
import rclpy
import numpy as np
import numpy.matlib 
import quaternion
import keyboard

from rclpy.node import Node
from numpy import asarray, savetxt, loadtxt
from ros2_igtl_bridge.msg import Transform
from trajcontrol_interfaces.srv import GetPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from scipy.ndimage import median_filter

class Registration(Node):

    def __init__(self):
        super().__init__('registration')

        #Declare node parameters
        self.declare_parameter('registration',1) # Registration parameter: 0 = load previous / 1 = obtain new
        
        #Topics from Aurora sensor node
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Service: Provide registration transform (obtain new or load previous)
        self.srv = self.create_service(GetPose, '/needle/stage_registration', self.registration_callback)

        self.registration = np.empty(shape=[0,7])
        self.Z_sensor = np.empty(shape=[0,7])
        self.Z_sensor = np.array([[1,2,3,1,0,0,0]])
        
        #Testing for non-blocking keyboard 
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('The keyboard is NOT blocking the node')

    # Get current Aurora sensor measurements
    def aurora_callback(self, msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        if name=="NeedleToTracker": # Name is adjusted in Plus .xml
            # Get aurora new reading
            self.Z_sensor = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])

    # Get registration transform
    def registration_callback(self, request, response):
        # Check if should make registration or load previous transform
        if(self.get_parameter('registration').get_parameter_value().integer_value == 1): # Calculate new registration transform
            self.get_logger().info('=== Registration Procedure ===')
            ###############################################################################
            ### TODO: Define B: Registration points in the stage frame                  ###
            ### B = np.array([[x0, x1, ..., xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]) ###
            ###############################################################################
            A = np.empty(shape=[3,0])               # registration points in aurora frame
            B = np.array([[1, 2], [1, 2], [1, 2]])  # registration points in stage frame
            
            # Get all registration points
            i = 1
            while (A.shape[1] < B.shape[1]):  
                self.get_logger().info('Please, place the sensor at Registration Point #%i and press key p' % (i))
                keyboard.wait('p')
                if len(self.Z_sensor)==0: #No points stored
                    self.get_logger().info('There is no sensor reading to store')
                else:
                    P = self.Z_sensor[0,0:3]
                    self.get_logger().info('Stored Point #%i = %s' % (i, P.T))
                    A = np.column_stack((A, P.T))
                    i += 1

            # Calculate registration transform
            self.registration = find_registration(A, B)
            self.get_logger().info('Registration transform ... \n registration = %s' %  (self.registration))

            # Save matrix to file
            savetxt(os.path.join('src','trajcontrol','files','registration.csv'), asarray(self.registration), delimiter=',')
        else:    # Load previous registration from file
            self.get_logger().info('Use previous registration')
            try:
                self.registration = loadtxt(os.path.join('src','trajcontrol','files','registration.csv'), delimiter=',')

            except IOError:
                self.get_logger().info('Could not find registration.csv file')
            
            self.get_logger().info('Loading stored registration transform ... \n registration = %s' %  (self.registration))

        # Prepare response message
        response.x=self.registration[0]
        response.y=self.registration[1]
        response.z=self.registration[2]
        response.qw=self.registration[3]
        response.qx=self.registration[4]
        response.qy=self.registration[5]
        response.qz=self.registration[6]
        return response

########################################################################
### Auxiliar functions ###
########################################################################

# Function: find_registration
# DO: From two sets of N 3D points in two different reference frames, find the best fit
#       in the LS-sense for the transformation between them (translation and rotation in quaternion)
# Inputs: 
#   A: set of N 3D points in first frame (numpy array 3xN)
#   B: set of N 3D points in second frame (numpy array 3xN)
# Output:
#   x_reg: transformation from first to second frame (numpy array [x, y, z, qw, qx, qy, qz])
def find_registration(A, B):
    [d, n] = np.shape(A)

    #Mean Center Data
    Ac = np.mean(A.T,axis=0)
    Bc = np.mean(B.T,axis=0)
    A = A - np.matlib.repmat(Ac[:,None], 1, n)
    B = B - np.matlib.repmat(Bc[:,None], 1, n)

    #Calculate Optimal Rotation
    M = np.matmul(A, B.T)
    N = np.array([[M[0,0]+M[1,1]+M[2,2], M[1,2]-M[2,1],        M[2,0]-M[0,2],        M[0,1]-M[1,0]],\
                [M[1,2]-M[2,1],        M[0,0]-M[1,1]-M[2,2], M[0,1]+M[1,0],        M[2,0]+M[0,2]],\
                [M[2,0]-M[0,2],        M[0,1]+M[1,0],        M[1,1]-M[0,0]-M[2,2], M[1,2]+M[2,1]],\
                [M[0,1]-M[1,0],        M[2,0]+M[0,2],        M[1,2]+M[2,1],        M[2,2]-M[0,0]-M[1,1]]])
    [w,v]=np.linalg.eig(N)
    ind=np.argmax(w)
    q = v[:,ind]                                                                #Rotation quaternion
    R = (q[0]**2-np.inner(q[1:4],q[1:4]))*np.eye(3) + 2*np.outer(q[1:4],q[1:4]) + \
        2*q[0]*np.array([[0,-q[3],q[2]],[q[3],0,-q[1]],[-q[2],q[1],0]])         #Rotation matrix

    #Calculate Optimal Translation
    t = Bc - np.matmul(R,Ac)

    x_reg = [t[0], t[1], t[2], q[0], q[1], q[2], q[3]] #final registration transform
    return x_reg

########################################################################

def main(args=None):
    rclpy.init(args=args)

    registration = Registration()

    rclpy.spin(registration)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    registration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
