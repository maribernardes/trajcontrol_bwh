import os
import rclpy

from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class TwoSensors(Node):

    def __init__(self):
        super().__init__('two_sensors')

        #Topics from Aurora sensor node
        self.subscription_aurora = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_aurora # prevent unused variable warning

        # #Published topics
        self.publisher_tip = self.create_publisher(PoseStamped, '/aurora/needle', 10)
        self.publisher_depth = self.create_publisher(PoseStamped, '/aurora/base', 10)

    # Get current Aurora sensor measurements
    def aurora_callback(self, msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        msg = PoseStamped()
        msg.header.frame_id = 'aurora'
        msg.pose.position = Point(x=msg_sensor.transform.translation.x, y=msg_sensor.transform.translation.y, z=msg_sensor.transform.translation.z)
        msg.pose.orientation = Quaternion(w=msg_sensor.transform.rotation.w, x=msg_sensor.transform.rotation.x, y=msg_sensor.transform.rotation.y, z=msg_sensor.transform.rotation.z)
        if name=="NeedleToTracker": # Name is adjusted in Plus .xml
            self.publisher_tip.publish(msg)
            self.get_logger().info('Needle = %s' % (msg.pose))
        if name=="BaseToTracker":
            self.publisher_depth.publish(msg)
            self.get_logger().info('Base = %s' % (msg.pose))


def main(args=None):
    rclpy.init(args=args)
    two_sensors = TwoSensors()

    rclpy.spin(two_sensors)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    two_sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
