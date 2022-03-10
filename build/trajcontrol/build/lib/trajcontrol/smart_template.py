import rclpy
import numpy as np
import ament_index_python 
import serial
import time


from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from stage_control_interfaces.action import MoveStage

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat
from std_msgs.msg import Int8

from datetime import datetime
now = datetime.now()

MM_2_COUNT = 500
COUNT_2_MM = 2.5349/2000.0

class VirtualRobot(Node):

    def __init__(self):
        super().__init__('virtual_robot')
        


        #Published topics
        self.publisher_needle_pose = self.create_publisher(PoseStamped, '/stage/state/needle_pose', 10)
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_needlepose_callback)

        #Topics from Insertion node
        self.subscription_insertion = self.create_subscription(Int8, '/needle/state/insertion', self.insertion_callback, 10)
        self.subscription_insertion  # prevent unused variable warning

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
               self.get_logger().info('Could not open Serial connection')
       
        
        self.needle_pose = 0.0
        self.time_stamp = 0.0
        self.i = 0

    def getMotorPosition(self):
        try:
            self.ser.flushInput()
            time.sleep(0.5)
            self.ser.write(str.encode("TP;"))
            time.sleep(0.1)
            bytesToRead = self.ser.inWaiting()
            data_temp = self.ser.read(bytesToRead-3)
            print(data_temp)
        except:
            self.status = 0
            return str(0)
        return data_temp

    def insertion_callback(self,msg_ins):
        self.insertion = msg_ins.data
        self.get_logger().info('PM Listening Insertion Node %f mm'  % (self.insertion))
        
        
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"


        read_position = self.getMotorPosition()

        Z = [0,0,0] #read_position.split(',')

        msg.pose.position.x = float(Z[0])*COUNT_2_MM
        msg.pose.position.y = float(self.insertion)
        msg.pose.position.z = float(Z[1])*COUNT_2_MM
        print("test antes antes")
        msg.pose.orientation = Quaternion(x=float(0), y=float(0), z=float(0), w=float(0))
        print("test antes")
        self.publisher_needle_pose.publish(msg)
        print("test")
        self.get_logger().info('Publish - Needle pose %i: x=%f, y=%f, z=%f, q=[%f, %f, %f, %f] in %s frame'  % (self.i, msg.pose.position.x, \
            msg.pose.position.y, msg.pose.position.z,  msg.pose.orientation.x, \
            msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, msg.header.frame_id))
        self.i += 1


    # Publish current needle_pose
    def needlepose_callback(self):

        msg = PoseStamped()
        msg.header.stamp = now
        msg.header.frame_id = "stage"


        read_position = self.getMotorPosition()

        Z = read_position.split(',')

        msg.pose.position.x = float(Z[0])*COUNT_2_MM
        msg.pose.position.y = self.insertion
        msg.pose.position.z = float(Z[1])*COUNT_2_MM

        msg.pose.orientation = Quaternion(x=float(0), y=float(0), z=float(0), w=float(0))

        self.publisher_needle_pose.publish(msg)
        self.get_logger().info('Publish - Needle pose %i: x=%f, y=%f, z=%f, q=[%f, %f, %f, %f] in %s frame'  % (self.i, msg.pose.position.x, \
            msg.pose.position.y, msg.pose.position.z,  msg.pose.orientation.x, \
            msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, msg.header.frame_id))
        self.i += 1

    # Destroy de action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Accept or reject a client request to begin an action
    # This server allows multiple goals in parallel
    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
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
            self.get_logger().info("Sent BG to Galil")
            return 1
        except:
            self.get_logger().info("*** could not send exec command ***")
            return 0

    def send_movement_in_counts(self,X,Channel):
   #     try:
        print(X)
        send = "PR%s=%d;" % (Channel,int(X))
        print(send)
        self.ser.write(str.encode(send))
        time.sleep(0.1)
        self.get_logger().info("Sent to Galil PR%s=%d" % (Channel,X))
    #    except:
    #        self.get_logger().info("*** could not send command ***")
    #        return 0


    # Execute a goal
    # This is a dummy action: the "goal" is to increment x from 0 to 4
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MoveStage.Feedback()
        feedback_msg.x = 0.0

        # Start executing the action
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return MoveStage.Result()

        my_goal = goal_handle.request
        # Update control input
        self.send_movement_in_counts(my_goal.x*MM_2_COUNT,"A")
        self.send_movement_in_counts(my_goal.z*MM_2_COUNT, "B")
        self.exec_motion()


        feedback_msg.x = float(0.0)

        self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.x))

        # Publish the feedback
        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()

        # Populate result message
        result = MoveStage.Result()
        result.x = feedback_msg.x

        self.get_logger().info('Returning result: {0}'.format(result.x))

        return result


def main(args=None):
    rclpy.init(args=args)

    virtual_robot = VirtualRobot()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(virtual_robot, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
