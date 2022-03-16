import rclpy
import ament_index_python 
import csv
import keyboard


from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ros2_igtl_bridge.msg import Transform

from sensor_msgs.msg import Image

from std_msgs.msg import Int8

STEP = 5

class InsertionNode(Node):

    def __init__(self):
        super().__init__('save_file')
        self.publisher_insertion = self.create_publisher(Int8, '/needle/state/insertion', 10)
        self.msg = Int8()
        
        self.msg.data=0
        self.publisher_insertion.publish(self.msg)

        keyboard.on_press_key('f', self.init_insertion)
        keyboard.on_press_key('s', self.step_insertion)  
        

    def init_insertion(self,event):
        self.msg.data=0
        self.publisher_insertion.publish(self.msg)
        self.get_logger().info('Init insertion')

    def step_insertion(self,event):
        self.msg.data=self.msg.data+STEP
        self.publisher_insertion.publish(self.msg)
        self.get_logger().info('Step insertion: %i' % (self.msg.data))
 


def main(args=None):
    rclpy.init(args=args)

    save_file = InsertionNode()

    rclpy.spin(save_file)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    save_file.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
