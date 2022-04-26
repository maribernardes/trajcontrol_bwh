import rclpy
import os
import csv
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from ros2_igtl_bridge.msg import Transform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from numpy import asarray


class SaveFile(Node):

    def __init__(self):
        super().__init__('save_file')
        
        #Declare node parameters
        self.declare_parameter('filename', 'my_data') #Name of file where data values are saved
        self.filename = os.path.join(os.getcwd(),'src','trajcontrol','data',self.get_parameter('filename').get_parameter_value().string_value + '.csv') #String with full path to file

        #Topics from aurora node (NeedleToTracker and BaseToTracker sensors)
        self.subscription_aurora = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_aurora # prevent unused variable warning

        #Topics from sensor processing node
        self.subscription_sensortip = self.create_subscription(PoseStamped, '/sensor/tip_filtered', self.sensortip_callback, 10)
        self.subscription_sensortip # prevent unused variable warning
        self.subscription_sensorbase = self.create_subscription(PoseStamped, '/sensor/base_filtered', self.sensorbase_callback, 10)
        self.subscription_sensorbase # prevent unused variable warning
        self.subscription_UI = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_UI  # prevent unused variable warning

        #Topics from estimator_node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.estimator_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Topics from controller_node
        self.subscription_controller = self.create_subscription(PointStamped, '/stage/control/cmd', self.control_callback, 10)
        self.subscription_controller  # prevent unused variable warning

        
        #Published topics
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.write_file_callback)

        #Array of data
        header = ['Timestamp sec', 'Timestamp nanosec', \
            'Entry_point x', 'Entry_point y', 'Entry_point z', \
            'AuroraTip x', 'AuroraTip y', 'AuroraTip z', 'AuroraTip qw', 'AuroraTip qx', 'AuroraTip qy', 'AuroraTip qz',
            'Tip x', 'Tip y', 'Tip z', 'Tip qw', 'Tip qx', 'Tip qy', 'Tip qz',
            'AuroraBase x', 'AuroraBase y', 'AuroraBase z', 'AuroraBase qw', 'AuroraBase qx', 'AuroraBase qy', 'AuroraBase qz',
            'Base x', 'Base y', 'Base z', 'Base qw', 'Base qx', 'Base qy', 'Base qz',
            'J00', 'J01', 'J02', 'J03', 'J04', 'J05', 'J06', \
            'J10', 'J11', 'J12', 'J13', 'J14', 'J15', 'J16', \
            'J20', 'J21', 'J22', 'J23', 'J24', 'J25', 'J26', \
            'J30', 'J31', 'J32', 'J33', 'J34', 'J35', 'J36', \
            'J40', 'J41', 'J42', 'J43', 'J44', 'J45', 'J46', \
            'J50', 'J51', 'J52', 'J53', 'J54', 'J55', 'J56', \
            'J60', 'J61', 'J62', 'J63', 'J64', 'J65', 'J66', \
            'Control_x', 'Control_z'    
        ]
        
        with open(self.filename, 'w', newline='', encoding='UTF8') as f: # open the file in the write mode
            writer = csv.writer(f)  # create the csv writer
            writer.writerow(header) # write a row to the csv file

        #Last data received
        self.entry_point = [0,0,0]          #skin entry point
        self.aurora_tip = [0,0,0,0,0,0,0]   #aurora tip data
        self.aurora_base = [0,0,0,0,0,0,0]  #aurora base data
        self.Z = [0,0,0,0,0,0,0]        #needle tip (filtered and transformed to robot frame)
        self.X = [0,0,0,0,0,0,0]        #robot/needle base
        self.J = [0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0]    #jacobian matrix
        self.cmd = [0,0]
        self.get_logger().info('Log data will be saved at %s' %(self.filename))   

    #Get current entry_point
    def entry_point_callback(self, msg):
        self.entry_point = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    #Get Aurora data
    def aurora_callback(self, msg):
        aurora = msg.transform
        if msg.name=="NeedleToTracker": # Name is adjusted in Plus .xml
            self.aurora_tip = [aurora.translation.x, aurora.translation.y, aurora.translation.z, \
                aurora.rotation.w, aurora.rotation.x, aurora.rotation.y, aurora.rotation.z]
        if msg.name=="BaseToTracker": # Name is adjusted in Plus .xml
            self.aurora_base = [aurora.translation.x, aurora.translation.y, aurora.translation.z, \
                aurora.rotation.w, aurora.rotation.x, aurora.rotation.y, aurora.rotation.z]

    #Get current Z (filtered and in robot frame)
    def sensortip_callback(self, msg):
        tip = msg.pose
        self.Z = [tip.position.x, tip.position.y, tip.position.z, \
            tip.orientation.w, tip.orientation.x, tip.orientation.y, tip.orientation.z]
        #self.get_logger().info('Received Z = %s in %s frame' % (self.Z, msg.header.frame_id))

    #Get current X
    def sensorbase_callback(self, msg):
        base = msg.pose
        self.X = [base.position.x, base.position.y, base.position.z, \
            base.orientation.w, base.orientation.x, base.orientation.y, base.orientation.z]
        #self.get_logger().info('Received X = %s in %s frame' % (self.X, msg.header.frame_id))
        
    #Get current J
    def estimator_callback(self,msg):
        self.J = np.array(CvBridge().imgmsg_to_cv2(msg)).flatten()

    #Get current control output
    def control_callback(self,msg):
        self.cmd = [msg.point.x, msg.point.z]        
        
    #Save data do file
    def write_file_callback(self):
        now = self.get_clock().now().to_msg()
       
        data = [now.sec, now.nanosec, \
            self.entry_point[0], self.entry_point[1], self.entry_point[2], \
            self.aurora_tip[0], self.aurora_tip[1], self.aurora_tip[2], self.aurora_tip[3], self.aurora_tip[4], self.aurora_tip[5], self.aurora_tip[6], \
            self.Z[0], self.Z[1], self.Z[2], self.Z[3], self.Z[4], self.Z[5], self.Z[6], \
            self.aurora_base[0], self.aurora_base[1], self.aurora_base[2], self.aurora_base[3], self.aurora_base[4], self.aurora_base[5], self.aurora_base[6], \
            self.X[0], self.X[1], self.X[2], self.X[3], self.X[4], self.X[5], self.X[6], \
            self.J[0], self.J[1], self.J[2], self.J[3], self.J[4], self.J[5], self.J[6], \
            self.J[7], self.J[8], self.J[9], self.J[10], self.J[11], self.J[12], self.J[13], \
            self.J[14], self.J[15], self.J[16], self.J[17], self.J[18], self.J[19], self.J[20], \
            self.J[21], self.J[22], self.J[23], self.J[24], self.J[25], self.J[26], self.J[27], \
            self.J[28], self.J[29], self.J[30], self.J[31], self.J[32], self.J[33], self.J[34], \
            self.J[35], self.J[36], self.J[37], self.J[38], self.J[39], self.J[40], self.J[41], \
            self.J[42], self.J[43], self.J[44], self.J[45], self.J[46], self.J[47], self.J[48], \
            self.cmd[0], self.cmd[1]]
        
        with open(self.filename, 'a', newline='', encoding='UTF8') as f: # open the file in append mode
            writer = csv.writer(f) # create the csv writer
            writer.writerow(data)  # append a new row to the existing csv file     

def main(args=None):
    rclpy.init(args=args)

    save_file = SaveFile()

    rclpy.spin(save_file)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    save_file.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
