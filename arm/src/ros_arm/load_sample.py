import motor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import time

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
        self.load_sample = False

    def aruco_pose_callback(self, msg):
        # obtain position and posture information of Arco Marker
        
        for pose in msg.poses:
            position = pose.position
            orientation = pose.orientation

            # self.get_logger().info('Aruco Marker Pose:')
            self.get_logger().info('Position: x=%f, y=%f, z=%f' % (position.x, position.y, position.z))
            # self.get_logger().info('Orientation: x=%f, y=%f, z=%f, w=%f' % (orientation.x, orientation.y, orientation.z, orientation.w))
            if position.z < 0.065:
                print("back")
                self.drive.back()
            elif position.x > 0.018:
                print("go right")
                self.drive.turn_right()
            elif position.x < -0.018:
                print("go left")
                self.drive.turn_left()
            elif position.z > 0.087:
                print("go straight")
                self.drive.forward()
            else:
                print("stop")
                self.drive.stop()
                time.sleep(1)
                self.drive.arm_sep()
                self.load_sample = True
        


def main(args=None):
    rclpy.init(args=args)
    aruco_pose_subscriber = ArucoPoseSubscriber()
    rclpy.spin(aruco_pose_subscriber)
    aruco_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
