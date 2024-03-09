import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class KeyInputPublisher(Node):
    def __init__(self):
        super().__init__('key_input_publisher')
        self.publisher_ = self.create_publisher(Int32, 'arm_state', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key_input = input("Enter a key: ")
        msg = Int32()
        msg.data = key_input
        self.publisher_.publish(msg)  

def main(args=None):
    rclpy.init(args=args)
    key_input_publisher = KeyInputPublisher()
    rclpy.spin(key_input_publisher)
    key_input_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
