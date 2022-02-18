import os
import rclpy
import numpy as np

from rclpy.node import Node
from numpy import asarray, savetxt, loadtxt
from ros2_igtl_bridge.msg import Transform
from geometry_msgs.msg import PoseStamped
from scipy.ndimage import median_filter

class SensorProcessing(Node):

    def __init__(self):
        super().__init__('sensor_processing')

        #Declare node parameters
        self.declare_parameter('registration') # Registration parameter
        
        if(self.get_parameter('registration').get_parameter_value().integer_value == 1):
            registration(self)
        else:
            load_registration(self)

        #Topics from Aurora sensor node
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Published topics
        self.publisher_filtered = self.create_publisher(PoseStamped, '/needle/state/pose_filtered', 10)

        self.i=0
        self.aurora = np.empty(shape=[0,7])
        self.Z = np.empty(shape=[0,7])
    
    # Get current Aurora sensor measurements
    def aurora_callback(self, msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        if name=="NeedleToTracker": # Name is adjusted in Plus .xml
            # Save aurora reading
            new_row = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])
            self.aurora = np.row_stack((self.aurora, new_row))
                
            # Smooth the measurements with a median filter 
            n = self.aurora.size
            size_win = min(n, 500)
            if (size_win>0): 
                Z_filt = median_filter(self.aurora[n-size_win:n,:], size=(40,1)) #40 samples median filter (column-wise)
                Z = Z_filt[size_win-1,0]
            else:
                Z = new_row
            
            # Store last Z measurement
            self.Z = Z
            self.get_logger().info('Sample #%i: Z = %s in Aurora frame' % (self.i, self.Z.T))
            self.i += 1

            ##########################################
            # TODO: Aurora to Robot transformation
            ##########################################
            #x_orig = Transform()
            #x_orig.translation.x = x


            #x_robot = pose_transform(x_orig, x_tf)
            #p_robot = np.matmul(self.R, np.array([[x, y, z, 1]]).T)

            msg = PoseStamped()

            # Commented it out because Plus has no timestamp
            # msg.header.stamp = msg_sensor.header.stamp # Use same timestamp from Aurora

            msg.position = msg_sensor.transform.translation
            msg.orientation = msg_sensor.transform.rotation
            self.publisher_filtered.publish(msg)

def registration(self):
    self.get_logger().info('Registration Procedure')

    ##########################################
    # TODO: Registration procedure
    ##########################################

    self.R = np.array([ ( 1, 0, 0, 0),
                        ( 0, 1, 0, 0),
                        ( 0, 0, 1, 0),
                        ( 0, 0, 1, 0),
                        ( 0, 0, 0, 1)])
    ##########################################
    
    # Save matrix to file
    savetxt(os.path.join('src','trajcontrol','files','registration.csv'), asarray(self.R), delimiter=',')

def load_registration(self):
    self.get_logger().info('Use previous registration')
    try:
        self.R = loadtxt(os.path.join('src','trajcontrol','files','registration.csv'), delimiter=',')
    except IOError:
        self.get_logger().info('Could not find registration.csv file - Starting new registration now...')
        registration(self)

    self.get_logger().info('Loading previous registration matrix... \n R = %s' %  (self.R))

########################################################################
### Auxiliar functions ###

# Function: pose_transform
# DO: Transform pose to new reference frame
# Inputs: 
#   x_origin: pose in original reference frame (ros2_igtl_bridge.msg.Transform)
#   x_tf: transformation from original to new frame (ros2_igtl_bridge.msg.Transform)
# Output:
#   x_new: pose in new reference frame (ros2_igtl_bridge.msg.Transform)
def pose_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf.translation.x, x_tf.translation.y, x_tf.translation.z)
    q_tf= np.quaternion(x_tf.rotation.w, x_tf.rotation.x, x_tf.rotation.y, x_tf.rotation.z)

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig.translation.x, x_orig.translation.y, x_orig.translation.z)
    q_orig = np.quaternion(x_orig.rotation.w, x_orig.rotation.x, x_orig.rotation.y, x_orig.rotation.z)

    #Transform to new frame
    p_new = p_tf + q_tf*p_orig*q_tf.conj()
    q_new = q_tf*q_orig

    x_new = Transform()
    x_new.translation.x = p_new.x
    x_new.translation.y = p_new.y
    x_new.translation.z = p_new.z
    x_new.rotation.w = q_new.w
    x_new.rotation.x = q_new.x
    x_new.rotation.y = q_new.y
    x_new.rotation.z = q_new.z

    return x_new

def main(args=None):
    rclpy.init(args=args)

    sensor_processing = SensorProcessing()

    rclpy.spin(sensor_processing)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_processing.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
