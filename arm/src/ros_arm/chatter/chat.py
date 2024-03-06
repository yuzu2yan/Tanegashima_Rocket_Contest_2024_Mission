import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ChatNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Int32, 'chat', 10)
        self.subscription = self.create_subscription(Int32, 'chat', self.listener_callback, 10)
        self.get_logger().info("Chat Node initialized")

    def send_message(self, message):
        msg = Int32()
        msg.data = message
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def arm_talker(chat_node, rclpy):
    if rclpy.ok():
        chat_node.send_message(2)
        rclpy.spin(chat_node)
    chat_node.destroy_node()

if __name__ == '__main__':
    rclpy.init() 
    chat_node = ChatNode('chat_node')
    arm_talker(chat_node, rclpy)
    rclpy.shutdown()
