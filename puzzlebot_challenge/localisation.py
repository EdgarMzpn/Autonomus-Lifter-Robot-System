#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class Localisation(Node):
    def __init__(self):
        super().__init__('Odometry')

        # Initialize wheel variables
        self.wr = 0.0
        self.wl = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.l = 0.19
        self.r = 0.05
        
        # Starting pose for the puzzlebot
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0
        
        # Subscribers
        self.wl_sub = self.create_subscription(Float32, 'wl', self.cbWl, 1)
        self.wr_sub = self.create_subscription(Float32, 'wr', self.cbWr, 1)

        # Publishers 
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        self.start_time = self.get_clock().now()
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.odom_reading)


    def cbWr(self, msg):
        self.wr = msg.data

    def cbWl(self, msg):
        self.wl = msg.data

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q

    def odom_reading(self):

        #Get time difference 

        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9
        

        self.linear_speed = self.r * (self.wr + self.wl) / 2.
        self.angular_speed = self.r * (self.wr - self.wl) / self.l

        self.angle += self.angular_speed * self.dt
        self.angle = self.angle % 6.28
        self.positionx += self.linear_speed * np.cos(self.angle)
        self.positiony += self.linear_speed * np.sin(self.angle)

        odom = Odometry()
        odom.header.stamp = self.current_time 
        odom.pose.pose.position.x = self.positionx
        odom.pose.pose.position.y = self.positiony
        q = self.euler_to_quaternion(0,0,self.angle)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.linear_speed
        odom.twist.twist.angular.z = self.angular_speed
        self.odom_pub.publish(odom)
        self.start_time = self.get_clock().now()




def main(args=None):
    rclpy.init(args=args)
    odometry = Localisation()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown

if __name__ == "__main__":
    main()