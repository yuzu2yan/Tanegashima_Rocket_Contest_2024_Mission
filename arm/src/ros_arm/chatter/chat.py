import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Arm_Node(Node):

    def __init__(self):
        super().__init__('node_arm')
        self.publisher_ = self.create_publisher(Int32, 'number', 10)
        self.subscription = self.create_subscription(Int32, 'number', self.number_callback, 10)
        self.subscription
        self.counter = 0

    def number_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        if msg.data == 1:
            self.counter += 1
            if self.counter % 2 == 0:
                self.send_message(2)
                self.get_logger().info('Sent: "2"')

    def send_message(self, number):
        msg = Int32()
        msg.data = number
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
