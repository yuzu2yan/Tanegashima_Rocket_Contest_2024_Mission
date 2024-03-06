import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Arm_Node(Node):

    def __init__(self):
        self.num = 0
        super().__init__('node_arm')
        self.publisher_ = self.create_publisher(Int32, 'arm_pub', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.subscription = self.create_subscription(Int32, 'truck_pub', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.num += 1

    def timer_callback(self):
        msg = Int32()
        msg.data = self.num
        self.publisher_.publish(msg)
        self.get_logger().info('Sent: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Arm_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
