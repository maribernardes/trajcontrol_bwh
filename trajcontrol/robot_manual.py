import rclpy
import numpy as np
import time
import serial

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from stage_control_interfaces.action import MoveStage

MM_2_COUNT = 1170.8 #1088.9
COUNT_2_MM = 1.0/1170.8

DELTA_MM = 0.5  #Increment for manual movement

SAFE_LIMIT = 5.0

class RobotManual(Node):

    def __init__(self):
        super().__init__('robot_manual')

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

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


        #Initialize robot at current position
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

    def check_limits(self,X,Channel):
        if X > SAFE_LIMIT*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = SAFE_LIMIT*MM_2_COUNT
        elif X < -SAFE_LIMIT*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = -SAFE_LIMIT*MM_2_COUNT
        return X

    def send_movement_in_counts(self,X,Channel):
        X = self.check_limits(X,Channel)
        send = "PR%s=%d;" % (Channel,int(X))
        self.ser.write(str.encode(send))
        time.sleep(0.1)
        send = "BG%s;" % (Channel)
        self.ser.write(str.encode(send))
        time.sleep(0.1)        
        self.get_logger().info("Sent to Galil PR%s=%d" % (Channel,X))

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        cmd = np.zeros((2,1))
        if (msg.data == 50): # move down
            cmd[1] = -DELTA_MM
        elif (msg.data == 52): # move left
            cmd[0] = -DELTA_MM
        elif (msg.data == 54): # move right
            cmd[0] = DELTA_MM
        elif (msg.data == 56): # move up
            cmd[1] = DELTA_MM

        # Send command to stage
        self.send_movement_in_counts(float(cmd[0])*MM_2_COUNT,"A")
        # WARNING: Galil channel B inverted, that is why the my_goal is negative
        self.send_movement_in_counts(-float(cmd[1])*MM_2_COUNT,"B")

def main(args=None):
    rclpy.init(args=args)
    robot_manual = RobotManual()
    rclpy.spin(robot_manual)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_manual.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

