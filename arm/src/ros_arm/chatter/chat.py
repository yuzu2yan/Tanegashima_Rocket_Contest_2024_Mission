import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(Int32, 'chatter', self.number_callback, 10)
        self.subscription  # subscriptionを破棄しないようにするために変数に割り当てる
        self.publisher_ = self.create_publisher(Int32, 'chatter', 10)
        self.stop_publishing = False  # パブリッシュ停止フラグ

    def number_callback(self, msg):
        if msg.data == 1:
            self.stop_publishing = True

    def publish_number(self):
        msg = Int32()
        msg.data = 2
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    timer = node.create_timer(1.0, node.publish_number)  # 1秒ごとに数値2をパブリッシュ
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.stop_publishing:
            timer.cancel()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
