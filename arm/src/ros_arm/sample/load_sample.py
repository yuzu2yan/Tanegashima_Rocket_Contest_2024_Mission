import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class ArucoPoseSubscriber(Node):

    def __init__(self):
        super().__init__('load_sample_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def aruco_pose_callback(self, msg):
        # アルコマーカーの位置と姿勢情報を取得
        for pose in msg.poses:
            position = pose.position
            orientation = pose.orientation

            # 取得した情報を表示
            # self.get_logger().info('Aruco Marker Pose:')
            self.get_logger().info('Position: x=%f, y=%f, z=%f' % (position.x, position.y, position.z))
            # self.get_logger().info('Orientation: x=%f, y=%f, z=%f, w=%f' % (orientation.x, orientation.y, orientation.z, orientation.w))
            if position.z < 0.07:
                print("back")
            elif position.x > 0.05:
                print("go right")
            elif position.x < -0.05:
                print("go left")
            elif position.z > 0.12:
                print("go straight")
            else:
                print("stop")
                self.drive.arm_sep()
                while True:
                    print("waiting")
            
            
        


def main(args=None):
    rclpy.init(args=args)

    aruco_pose_subscriber = ArucoPoseSubscriber()

    rclpy.spin(aruco_pose_subscriber)

    aruco_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
