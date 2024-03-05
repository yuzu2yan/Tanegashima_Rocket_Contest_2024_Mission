import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class Teleop(Node):

    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def teleop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        key = None
        try:
            while key != '\x03':
                key = self.getKey()
                if key == 'w':
                    self.twist.linear.x = 0.5  # 前進
                elif key == 's':
                    self.twist.linear.x = -0.5  # 後退
                elif key == 'a':
                    self.twist.angular.z = 1.0  # 左回転
                elif key == 'd':
                    self.twist.angular.z = -1.0  # 右回転
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0

                self.publisher.publish(self.twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)

    teleop = Teleop()
    teleop.teleop()

    rclpy.spin(teleop)

    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
