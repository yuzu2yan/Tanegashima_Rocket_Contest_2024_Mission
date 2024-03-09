import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32

class Truck_Node(Node):

    def __init__(self):
        '''
        state 0: not landing
        state 1: landing and head to sample
        state 2: reach the sample and waiting
        state 3: load the sample and head to goal
        '''
        self.state = 0
        self.long = 0
        self.lat = 0
        super().__init__('node_truck')
        self.publisher_ = self.create_publisher(Int32, 'truck', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.state_subscription = self.create_subscription(Int32, 'arm_state', self.state_callback, 10)
        self.state_subscription
        self.locate_subscription = self.create_subscription(Float64MultiArray, 'arm_locate', self.locate_callback, 10)
        self.locate_subscription

    def state_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        if msg.data == 3:
            self.state = 3
    
    def locate_callback(self, msg):
        self.get_logger().info('longitude: "%s"' % msg.data[0] + ' latitude: "%s"' % msg.data[1])
        self.long = msg.data[0]
        self.lat = msg.data[1] 
        
    def timer_callback(self):
        msg = Int32()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info('Sent: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Truck_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
