import motor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import time
import chat

class ArucoPoseSubscriber(Node):

    def __init__(self):
        super().__init__('collect_sample_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.drive = motor.Motor()
        self.drive.stop()
        self.drive.arm_sep()
        self.collect_sample = False

    def aruco_pose_callback(self, msg):
        # obtain position and posture information of Arco Marker  
        if self.collect_sample != True:
            for pose in msg.poses:
                position = pose.position
                orientation = pose.orientation

                # self.get_logger().info('Aruco Marker Pose:')
                self.get_logger().info('Position: x=%f, y=%f, z=%f' % (position.x, position.y, position.z))
                # self.get_logger().info('Orientation: x=%f, y=%f, z=%f, w=%f' % (orientation.x, orientation.y, orientation.z, orientation.w))
                if position.z < 0.05:
                    self.get_logger().info('back')
                    self.drive.back()
                elif position.x > 0.02:
                    self.get_logger().info('turn right')
                    self.drive.turn_right()
                elif position.x < -0.02:
                    self.get_logger().info('turn left')
                    self.drive.turn_left()
                elif position.z > 0.075:
                    self.get_logger().info('forward')
                    self.drive.forward()
                else:
                    self.get_logger().info('stop')
                    self.drive.stop()
                    self.drive.down_arm()
                    self.drive.grabing()
                    time.sleep(1)
                    self.drive.rising_arm()
                    self.collect_sample = True
        


def main(args=None):
    rclpy.init(args=args)
    node = chat.Arm_Node()
    rclpy.spin(node)
    while node.state != 3:
        print("waiting")
        pass
    aruco_pose_subscriber = ArucoPoseSubscriber()
    rclpy.spin(aruco_pose_subscriber)
    aruco_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
