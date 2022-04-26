import os
import rclpy
import numpy as np
import ament_index_python 
import serial
import time
import quaternion

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from stage_control_interfaces.action import MoveStage
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
from scipy.ndimage import median_filter

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat
from std_msgs.msg import Int8

from datetime import datetime

MM_2_COUNT = 1088.9
COUNT_2_MM = 1.0/1088.9

class SmartTemplate(Node):

    def __init__(self):
        super().__init__('smart_template')      

        #Topics from Aurora sensor node
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning

        #Published topics
        timer_period = 0.8  # seconds
        self.timer = self.create_timer(timer_period, self.timer_needle_pose_callback)
        self.publisher_needle_pose = self.create_publisher(PoseStamped, '/stage/state/needle_pose', 10)

        #Action server
        self._action_server = ActionServer(self, MoveStage, '/move_stage', execute_callback=self.execute_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)

        #Start serial communication
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)  # open serial port
            self.connectionStatus = True
            self.get_logger().info('Serial connection open ttyUSB1')
            print(self.ser.is_open)
        except:
            try:
                self.ser = serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=1)  # open serial port
                self.get_logger().info('Serial connection open ttyUSB2')
                print(self.ser.is_open)
            except:
                try:
                    self.ser = serial.Serial('/dev/ttyUSB3', baudrate=115200, timeout=1)  # open serial port
                    self.get_logger().info('Serial connection open ttyUSB3')
                    print(self.ser.is_open)
                except:
                    self.get_logger().info('Could not open Serial connection')



        #Stored values
        self.entry_point = np.empty(shape=[0,7])    # Initial needle tip pose
        self.registration = np.empty(shape=[0,7])   # Registration transform (from aurora to stage)
        self.aurora = np.empty(shape=[0,7])         # All stored Aurora readings as they are sent
        self.needle_base = np.empty(shape=[0,7])    # Base sensor value (filtered and transformed to stage frame)

    def getMotorPosition(self):
        try:
            self.ser.flushInput()
            time.sleep(0.1)
            self.ser.write(str.encode("TP;"))
            time.sleep(0.5)
            bytesToRead = self.ser.inWaiting()
            data_temp = self.ser.read(bytesToRead-3)
        except:
            self.status = 0
            return str(0)
        return data_temp

    # Timer to publish '/stage/state/needle_pose'  
    def timer_needle_pose_callback(self):
        if (self.needle_base.size != 0): 
            # Read needle guide position from robot motors
            read_position = str(self.getMotorPosition())
            read_position = read_position[2 : : ]
            read_position = read_position.replace(':', '')
            Z = read_position.split(',')
            # Construct robot message to publish             
            # Add the initial entry point (home position)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "stage"
            msg.pose.position.x = float(Z[0])*COUNT_2_MM + self.entry_point[0,0]
            msg.pose.position.y = float(self.needle_base[1])
            # WARNING: Galil channel B inverted, that is why the my_goal is negative
            msg.pose.position.z = -float(Z[1])*COUNT_2_MM + self.entry_point[2,0]

            self.get_logger().info('motor read: %s %s ' % (Z[0],Z[2]))
  
            msg.pose.orientation = Quaternion(x=float(0), y=float(0), z=float(0), w=float(1))
            self.publisher_needle_pose.publish(msg)

            self.get_logger().info('needle_pose: x=%f, y=%f, z=%f, q=[%f, %f, %f, %f] in %s frame'  % (msg.pose.position.x, msg.pose.position.y, \
                msg.pose.position.z,  msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.header.frame_id))

    # Initialization after needle is positioned in the entry point (after SPACE hit)
    def entry_callback(self, msg):
        if (self.entry_point.size == 0):
            self.ser.write(str.encode("DPA=0;"))
            time.sleep(0.02)
            self.ser.write(str.encode("PTA=1;"))
            time.sleep(0.02)
            self.ser.write(str.encode("DPB=0;"))
            time.sleep(0.02)
            self.ser.write(str.encode("PTB=1;"))
            time.sleep(0.02)
            self.ser.write(str.encode("SH;")) #Check this code
            self.AbsoluteMode = True
            self.get_logger().info('Needle guide at position zero')

            # Store entry point
            entry_point = msg.pose
            self.entry_point = np.array([[entry_point.position.x, entry_point.position.y, entry_point.position.z, \
                                entry_point.orientation.w, entry_point.orientation.x, entry_point.orientation.y, entry_point.orientation.z]]).T

            # Load stored registration transform
            self.get_logger().info('Loading stored registration transform ...')
            try:
                self.registration = np.array(loadtxt(os.path.join(os.getcwd(),'src','trajcontrol','files','registration.csv'), delimiter=','))
            except IOError:
                self.get_logger().info('Could not find registration.csv file')
            self.get_logger().info('Registration = %s' %  (self.registration))

    # Get current Aurora sensor measurements
    # Filter measurement, transform to stage frame and store in self.needle_base
    def aurora_callback(self,msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        if name=="BaseToTracker": # Name is adjusted in Plus .xml
            # Get aurora new reading
            Z_sensor = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])

            # Filter and transform Aurora data only after registration was loaded from file
            if (self.registration.size != 0): 
                self.aurora = np.row_stack((self.aurora, Z_sensor))

                # Smooth the measurements with a median filter 
                n = self.aurora.shape[0]
                size_win = min(n, 500) #array window size
                if (size_win>0): 
                    Z_filt = median_filter(self.aurora[n-size_win:n,:], size=(40,1)) # use 40 samples median filter (column-wise)
                    Z_sensor = Z_filt[size_win-1,:]                                  # get last value
                            
                # Transform from sensor to robot frame
                self.needle_base = pose_transform(Z_sensor, self.registration)
                # self.get_logger().info('needle_base = %s' %  (self.needle_base))
                
    # Destroy de action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Accept or reject a client request to begin an action
    # This server allows multiple goals in parallel
    def goal_callback(self, goal_request):
        # self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    # Accept or reject a client request to cancel an action
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def exec_motion(self):
        try:
            self.ser.write(str.encode("BG \r"))
            time.sleep(0.1)
            self.ser.write(str.encode("PR 0,0,0,0 \r"))
            # self.get_logger().info("Sent BG to Galil")
            return 1
        except:
            self.get_logger().info("*** could not send exec command ***")
            return 0

    def check_limits(self,X,Channel):
        if X > 10*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = 10*MM_2_COUNT
        elif X < -10*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = -10*MM_2_COUNT
        return X

    def send_movement_in_counts(self,X,Channel):
   #     try:

        X = self.check_limits(X,Channel)
        send = "PA%s=%d;" % (Channel,int(X))
        self.ser.write(str.encode(send))
        time.sleep(0.1)
        self.get_logger().info("Sent to Galil PA%s=%d" % (Channel,X))
    #    except:
    #        self.get_logger().info("*** could not send command ***")
    #        return 0


    # Execute a goal
    async def execute_callback(self, goal_handle):
        # self.get_logger().info('Executing goal...')

        feedback_msg = MoveStage.Feedback()
        #TODO
        feedback_msg.x = 0.0 #self.needle_base[0,0]
        feedback_msg.z = 0.0 #self.needle_base[1,0]

        # Start executing the action
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return MoveStage.Result()

        # Subtract entry point from goal because robot considers initial position to be (0,0)
        my_goal = goal_handle.request
        my_goal.x = my_goal.x - self.entry_point[0,0]
        my_goal.z = my_goal.z - self.entry_point[2,0]

        # Update control input
        self.send_movement_in_counts(my_goal.x*MM_2_COUNT,"A")
        # WARNING: Galil channel B inverted, that is why the my_goal is negative
        self.send_movement_in_counts(-my_goal.z*MM_2_COUNT,"B")


        feedback_msg.x = float(0.0)

        # self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.x))

        # Publish the feedback
        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()

        # Populate result message
        result = MoveStage.Result()
        result.x = feedback_msg.x

        # self.get_logger().info('Returning result: {0}'.format(result.x))

        return result

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

    smart_template = SmartTemplate()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(smart_template, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    smart_template.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()