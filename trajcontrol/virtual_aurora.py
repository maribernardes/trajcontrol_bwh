import rclpy
import os
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ros2_igtl_bridge.msg import Transform
from scipy.io import loadmat

class VirtualAurora(Node):

    def __init__(self):
        super().__init__('virtual_aurora')

        #Declare node parameters
        self.declare_parameter('dataset', 'aurora_26') #Dataset file name

        #Published topics
        #Topics from Aurora sensor node
        self.publisher = self.create_publisher(Transform, 'IGTL_TRANSFORM_IN', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Load data from matlab file
        file_path = os.path.join(os.getcwd(),'src','trajcontrol','files',self.get_parameter('dataset').get_parameter_value().string_value + '.mat') #String with full path to file
        trial_data = loadmat(file_path, mat_dtype=True)
        self.sensor = trial_data['sensor'][0]
        self.i=0

    def timer_callback(self):        

        msg = Transform()
        msg.name = "NeedleToTracker"    

        if (self.i < self.sensor.size):
            X = self.sensor[self.i]
            j = int((X.size/3)-1)
            msg.transform.translation.x = float(X[0][j])
            msg.transform.translation.y = float(X[1][j])
            msg.transform.translation.z = float(X[2][j])

            self.i += 1

        self.publisher.publish(msg)
        #self.get_logger().info('Tip Position = [%f, %f, %f] ' % (msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z))

def main(args=None):
    rclpy.init(args=args)

    virtual_aurora = VirtualAurora()

    rclpy.spin(virtual_aurora)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_aurora.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
