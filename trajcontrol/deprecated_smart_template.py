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

MM_2_COUNT = 1170.8 #1088.9
COUNT_2_MM = 1.0/1170.8

LIMIT = 5.0

class SmartTemplate(Node):

    def __init__(self):
        super().__init__('smart_template')      

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning

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
        if X > LIMIT*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = LIMIT*MM_2_COUNT
        elif X < -LIMIT*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = -LIMIT*MM_2_COUNT
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
        if (self.entry_point.size != 0): #Entry point already defined
            feedback_msg = MoveStage.Feedback()
            # TODO: Feedback messages
            feedback_msg.x = 0.0
            feedback_msg.z = 0.0

            # Start executing the action
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return MoveStage.Result()

            # Subtract entry point from goal because robot considers initial position to be (0,0)
            my_goal = goal_handle.request
            my_goal.x = my_goal.x - self.entry_point[0,0]
            my_goal.z = my_goal.z - self.entry_point[2,0]

            self.get_logger().info("Controller = %f %f" % (my_goal.x,my_goal.z))
            # Update control input
            #self.send_movement_in_counts(my_goal.x*MM_2_COUNT,"A")
            # WARNING: Galil channel B inverted, that is why the my_goal is negative
            #self.send_movement_in_counts(-my_goal.z*MM_2_COUNT,"B")

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
