#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


class localisation:
    def __init__(self):
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
        self.wl_sub = rospy.Subscriber('wl', Float32, self.cbWl)
        self.wr_sub = rospy.Subscriber('wr', Float32, self.cbWr)

        # Publishers 
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size = 10)

        self.start_time = rospy.get_time()
        time_period = 0.1


    def cbWr(self, msg):
        self.wr = msg.data

    def cbWl(self, msg):
        self.wl = msg.data

    def odom_reading(self):
        #Get time difference 

        self.current_time = self.rospy.get_time()
        self.duration = self.current_time - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9
        

        self.linear_speed = self.r * (self.wr + self.wl) / 2.
        self.angular_speed = self.r * (self.wr - self.wl) / self.l

        self.angle += self.angular_speed * self.dt
        self.angle = self.angle % 6.28
        self.positionx += self.linear_speed * np.cos(self.angle) * self.dt
        self.positiony += self.linear_speed * np.sin(self.angle) * self.dt

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
        self.start_time = self.current_time


def main(args=None):
    rospy.init_node("localisation")
    odometry = localisation()
    rate = rospy.Rate(100)
    rospy.spin()


if __name__ == "__main__":
    main()
