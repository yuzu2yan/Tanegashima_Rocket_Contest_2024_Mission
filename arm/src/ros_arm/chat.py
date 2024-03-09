import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
import gnss

class Arm_Node(Node):

    def __init__(self):
        '''
        state 0: not landing
        state 1: landing and head to sample
        state 2: reach the sample and waiting
        state 3: load the sample and head to goal
        '''
        self.state = 0
        self.long = 130
        self.lat = 10
        super().__init__('node_arm')
        self.state_publisher = self.create_publisher(Int32, 'arm_state', 10)
        self.timer_1 = self.create_timer(0.5, self.timer1_callback)
        self.locate_publisher = self.create_publisher(Float64MultiArray, 'arm_locate', 10)
        self.timer_2 = self.create_timer(0.5, self.timer2_callback)
        self.subscription = self.create_subscription(Int32, 'truck', self.state_callback, 10)
        self.subscription

    def state_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        if msg.data == 1:
            self.state = 1
        if msg.data == 2:
            self.state = 2

    def timer1_callback(self):
        msg = Int32()
        msg.data = self.state
        self.state_publisher.publish(msg)
        self.get_logger().info('Sent: "%s"' % msg.data)
        
    def timer2_callback(self):
        msg = Float64MultiArray()
        msg.data = gnss.read_GPSData()
        self.state_publisher.publish(msg)
        self.get_logger().info('Sent: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Arm_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
