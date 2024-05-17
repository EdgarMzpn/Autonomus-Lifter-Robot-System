#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf_transformations import quaternion_from_euler

class Localisation(Node):
    def __init__(self):
        super().__init__('Odometry')

        # Initialize wheel variables
        self.wr = 0.0               # Right Wheel
        self.wl = 0.0               # Left Wheel
        self.linear_speed = 0.0     # Linear Speed
        self.angular_speed = 0.0    # Angular Speed
        self.l = 0.19               # Wheelbase
        self.r = 0.05               # Radius of the Wheel
        
        # Starting pose for the puzzlebot
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0

        self.true_angle = 0.0
        
        # Subscribers
        self.sub_pose = self.create_subscription(Pose, '/pose', self.cbPose, 10)
        self.sub_wl = self.create_subscription(Float32, '/VelocityEncL', self.cbWl,         
        rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.sub_wr = self.create_subscription(Float32, '/VelocityEncR', self.cbWr,
        rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))

        # Publishers 
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        # Start the timer now
        self.start_time = self.get_clock().now()
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.odom_reading)

    def cbPose(self, msg):
        self.true_angle = msg.orientation.z


    def cbWr(self, msg):
        self.wr = msg.data

    def cbWl(self, msg):
        self.wl = msg.data

    def odom_reading(self):

        #Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9
        
        self.linear_speed = self.r * (self.wr + self.wl) / 2.
        self.angular_speed = self.r * (self.wr - self.wl) / self.l

        self.angle = self.angle % 6.28
        self.get_logger().info("Angle variable: {}".format(self.angle))
        self.get_logger().info("Angle true: {}".format(self.true_angle))
        self.angle += self.angular_speed * self.dt
        self.positionx += self.linear_speed * np.cos(self.angle) * self.dt
        self.positiony += self.linear_speed * np.sin(self.angle) * self.dt

        # Publish odometry via odom topic
        odom = Odometry()
        odom.header.stamp = self.current_time 
        odom.pose.pose.position.x = self.positionx
        odom.pose.pose.position.y = self.positiony  
        q = quaternion_from_euler(0., 0., self.angle)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.linear_speed
        odom.twist.twist.angular.z = self.angular_speed
        self.odom_pub.publish(odom)
        # self.get_logger().info("Position y msg: {}".format(odom.pose.pose.position.y))
        self.start_time = self.get_clock().now()




def main(args=None):
    rclpy.init(args=args)
    odometry = Localisation()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()