import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gnss

class Talker(Node):

    def __init__(self):
        super().__init__('arm_talker')
        self.publisher_ = self.create_publisher(Int32, 'chatter', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.msg_ = Int32
        
    def timer_callback(self):
        msg = Int32()
        msg.data = 2
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

class Listener(Node):

    def __init__(self):
        super().__init__('arm_listener')
        self.subscription_ = self.create_subscription(
            Int32, 'chatter', self.listener_callback, 10)
        self.subscription_

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


if __name__ == '__main__':
    rclpy.init()
    listener = Listener()
    rclpy.spin(listener)
    rclpy.shutdown()

