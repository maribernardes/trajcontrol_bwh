import os
import rclpy
import numpy as np
import time
import keyboard
import numpy.matlib 
import quaternion

from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from scipy.ndimage import median_filter

class SensorProcessing(Node):

    def __init__(self):
        super().__init__('sensor_processing')

        #Declare node parameters
        self.declare_parameter('registration',1) # Registration parameter: 0 = load previous / 1 = obtain new

        #Topics from Aurora sensor node
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Published topics
        self.publisher_filtered = self.create_publisher(PoseStamped, '/needle/state/pose_filtered', 10)

        # Registration points A (aurora) and B (stage)
        ###############################################################################
        ### TODO: Define B: Registration points in the stage frame                  ###
        ### B = np.array([[x0, x1, ..., xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]) ###
        ###############################################################################
        self.A = np.empty(shape=[3,0])               # registration points in aurora frame
        self.B = np.array([[1, 2], [1, 2], [1, 2]])  # registration points in stage frame
        self.point_count = 1

        #Get events from keyboard
        self.keyboard_loop = True
        keyboard.on_press_key('return', self.get_point)

        #Stored values
        self.registration = np.empty(shape=[0,7])   # Registration transform (from aurora to stage)
        self.aurora = np.empty(shape=[0,7])         # All stored Aurora readings as they are sent
        self.Z_sensor = np.empty(shape=[0,7])       # Aurora values as they are sent
        self.Z = np.empty(shape=[0,7])              # Filtered aurora data in robot frame
        #self.Z_sensor = np.array([[1, 2, 3]])       # REMOVE AFTER TESTS

    # Get current Aurora sensor measurements
    def aurora_callback(self, msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        #self.get_logger().info('Aurora callback') # REMOVE AFTER TESTS
        if name=="NeedleToTracker": # Name is adjusted in Plus .xml
            # Get aurora new reading
            self.Z_sensor = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])

            # Filter and transform Aurora data only after registration was performed or loaded from file
            if len(self.registration) != 0: 
                self.aurora = np.row_stack((self.aurora, self.Z_sensor))
                self.get_logger().info('Sample Z = %s in aurora frame' % (self.Z_sensor))

                # Smooth the measurements with a median filter 
                n = self.aurora.shape[0]
                size_win = min(n, 500) #array window size
                if (size_win>0): 
                    Z_filt = median_filter(self.aurora[n-size_win:n,:], size=(40,1)) # use 40 samples median filter (column-wise)
                    Z_sensor = Z_filt[size_win-1,:]                                  # get last value
                            
                # Transform from sensor to robot frame
                self.Z = pose_transform(Z_sensor, self.registration)
                
                # Publish last needle filtered pose in robot frame
                msg = PoseStamped()
                # msg.header.stamp = msg_sensor.header.stamp # Use same timestamp from Aurora (Commented it out because Plus has no timestamp)
                msg.header.frame_id = 'stage'
                msg.pose.position = Point(x=self.Z[0], y=self.Z[1], z=self.Z[2])
                msg.pose.orientation = Quaternion(w=self.Z[3], x=self.Z[4], y=self.Z[5], z=self.Z[6])
                self.publisher_filtered.publish(msg)
    
    def get_point(self,event):
        if len(self.Z_sensor)==0: #No points stored
            self.get_logger().info('There is no sensor reading to store')
        else:
            P = self.Z_sensor[0,0:3]
            self.get_logger().info('Stored Point #%i = %s' % (self.point_count, P.T))
            self.A = np.column_stack((self.A, P.T))
            self.point_count += 1

    def get_registration(self):    
        # Check if should make registration or load previous transform
        if(self.get_parameter('registration').get_parameter_value().integer_value == 1): # Calculate new registration transform
                     
            # Get points until A is same size as B
            if (self.A.shape[1] < self.B.shape[1]):  
                self.get_logger().info('Please, place the sensor at Registration Point #%i and press ENTER' % (self.point_count))
                time.sleep(0.5) #Not sure if necessary
            else:
                # Calculate registration transform
                self.registration = find_registration(self.A, self.B)   #Store registration transform
                keyboard.unhook_all()                                   #No more keyboard events

            # Save matrix to file
            savetxt(os.path.join('src','trajcontrol','files','registration.csv'), asarray(self.registration), delimiter=',')
        else:    # Load previous registration from file
            self.get_logger().info('Use previous registration')
            try:
                self.registration = loadtxt(os.path.join('src','trajcontrol','files','registration.csv'), delimiter=',')

            except IOError:
                self.get_logger().info('Could not find registration.csv file')
            
            self.get_logger().info('Loading stored registration transform ... \n registration = %s' %  (self.registration))

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

# Function: pose_transform
# DO: Transform pose to new reference frame
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf*q_orig
    p_new = q_tf*p_orig*q_tf.conj() + p_tf

    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

def main(args=None):
    rclpy.init(args=args)

    sensor_processing = SensorProcessing()

    while rclpy.ok():
        rclpy.spin_once(sensor_processing, timeout_sec=0.5)
        if len(sensor_processing.registration) == 0: #No registration yet
            sensor_processing.get_registration()
        else:
            sensor_processing.get_logger().info('Registration transform ... \n registration = %s' %  (sensor_processing.registration))
            break

    rclpy.spin(sensor_processing)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_processing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
