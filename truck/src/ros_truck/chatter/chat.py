import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Int32, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_number)  # 1秒ごとに数値1をパブリッシュ

    def publish_number(self):
        msg = Int32()
        msg.data = 1
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
