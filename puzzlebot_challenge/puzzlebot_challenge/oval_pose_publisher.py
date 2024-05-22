import time
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.pose_pub = self.create_publisher(Pose, '/pose_ideal', 10)
        self.error_sub = self.create_subscription(Point, '/error', self.cb_error, 10)
        self.arrive_sub = self.create_subscription(Bool, '/arrive', self.cb_arrive, 10)
        self.pose_msg = Pose()
        self.pose_pub.publish(self.pose_msg)
        timer_period = 0.1  # seconds
        self.start_time = self.get_clock().now()
        self.counter = 0
        self.timer = self.create_timer(timer_period, self.publish_pose)
        self.count = 0

    def cb_error(self, msg: Point):
        self.position_error = msg.x

    def cb_arrive(self, msg: Bool):
        self.arrive = msg.data

    def publish_pose(self):
        # self.current_time = self.get_clock().now()

        # # Convert the duration to a float value (in seconds)
        # self.dt = self.current_time.nanoseconds * 1e-9

        # Parameters for the circle
        # h, k = 0, 0  # Center of the circle
        r = 1  # Radius of the circle

        # Create an array of angles from 0 to 2*pi
        theta = np.linspace(0, 2 * np.pi, 100)

        # Update x and y values
        if self.arrive:
            # self.pose_msg.position.x = 2*np.sin(count)  # Replace with your x value
            # self.pose_msg.position.y = np.cos(count)  # Replace with your y value
            # Parametric equations for the circle
            self.pose_msg.position.x = 2 * r * np.cos(theta[self.count])
            self.pose_msg.position.y = r * np.sin(theta[self.count])
            
            if self.count >= len(theta):
                self.count = 0
            else:
                self.count += 1

        self.pose_pub.publish(self.pose_msg)
        # self.get_logger().info('Publishing ideal pose: x=%f, y=%f' % (self.pose_msg.position.x, self.pose_msg.position.y))

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()