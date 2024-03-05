import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import gnss

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'chatter', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.msg_ = Float64MultiArray

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = gnss.read_GPSData()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
