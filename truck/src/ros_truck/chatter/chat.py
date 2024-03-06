import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ChatNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Int32, 'chattter', 10)
        self.timer_ = None
        self.subscription = self.create_subscription(Int32, 'chattter', self.listener_callback, 10)
        self.get_logger().info("Chat Node initialized")

    def send_message(self, message):
        msg = Int32()
        msg.data = message
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def truck_talker(chat_node, rclpy):
    if rclpy.ok():
        chat_node.timer_ = chat_node.create_timer(0.5, chat_node.send_message(1))
        rclpy.spin(chat_node)
    chat_node.destroy_node()

if __name__ == '__main__':
    rclpy.init() 
    chat_node = ChatNode('chat_node')
    truck_talker(chat_node, rclpy)
    rclpy.shutdown()
