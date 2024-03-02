import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription_ = self.create_subscription(
            Float64MultiArray, 'chatter', self.listener_callback, 10)
        self.subscription_

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
