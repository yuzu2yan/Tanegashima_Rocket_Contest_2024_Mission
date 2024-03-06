import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Arm_Node(Node):

    def __init__(self):
        self.num = 0
        super().__init__('node_arm')
        self.publisher_ = self.create_publisher(Int32, 'arm_pub', 10)
        self.timer_ = self.create_timer(0.5, self.send_message)
        self.subscription = self.create_subscription(Int32, 'arm_sub', self.receive_message, 10)
        self.subscription

    def receive_message(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        if msg.data == 1:
            self.num += 1
            self.get_logger().info('Sent:', msg.data)

    def send_message(self):
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
