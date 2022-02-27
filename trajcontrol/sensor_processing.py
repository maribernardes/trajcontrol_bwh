import rclpy
import numpy as np

from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform
from trajcontrol_interfaces.srv import GetPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from scipy.ndimage import median_filter

class SensorProcessing(Node):

    def __init__(self):
        super().__init__('sensor_processing')
        
        #Topics from Aurora sensor node
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Service call from registration node
        self.cli = self.create_client(GetPose, '/needle/stage_registration')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Registration service not available, waiting again...')
        self.req = GetPose.Request()

        self.get_logger().info('Waiting service for loading registration transform')
        self.future = self.cli.call_async(self.req) # Make asynchonous call for service

        #Published topics
        self.publisher_filtered = self.create_publisher(PoseStamped, '/needle/state/pose_filtered', 10)

        #Stored values
        self.registration = np.empty(shape=[0,7])
        self.Z_sensor = np.empty(shape=[0,7])
        self.aurora = np.empty(shape=[0,7])         # Aurora readings as they are sent
        self.Z = np.empty(shape=[0,7])              # Filtered data in robot frame
    

    # Get current Aurora sensor measurements
    def aurora_callback(self, msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        if name=="NeedleToTracker": # Name is adjusted in Plus .xml
            # Get aurora new reading
            self.Z_sensor = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])

            # Filter and transform Aurora data only after registration was performed or loaded from file
            if len(self.registration) != 0: 
                self.aurora = np.row_stack((self.aurora, self.Z_sensor))
                self.get_logger().info('Sample Z = %s in aurora frame' % (Z_sensor))

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
            

########################################################################
### Auxiliar functions ###
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
        rclpy.spin_once(sensor_processing)
        if sensor_processing.future.done():
            try:
                 response = sensor_processing.future.result()
            except Exception as e:
                sensor_processing.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                sensor_processing.registration = np.array([response.x, response.y, response.z, response.qw, response.qx, response.qy, response.qz])
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
