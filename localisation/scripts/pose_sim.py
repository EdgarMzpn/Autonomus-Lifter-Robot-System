#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose

class Pose_Sim:
    def __init__(self):
        rospy.init_node('Pose_Sim')
        # Initialize wheel variables
        self.wl = 0.0               # Left Wheel
        self.wr = 0.0               # Right Wheel
        self.linear_speed = 0.0     # Linear Speed
        self.angular_speed = 0.0    # Angular Speed
        self.l = 0.19               # Wheelbase
        self.r = 0.05               # Radius of the wheel

        # Starting pose for the puzzlebot
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0
        self.position = Pose()
        
        # Define the Publishers
        self.wl_pub = rospy.Publisher("wl", Float32, queue_size=1)
        self.wr_pub = rospy.Publisher("wr", Float32, queue_size=1)
        self.pose_pub = rospy.Publisher("pose", Pose, queue_size=1)

        # Define the Subscribers
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cbCmdVel)

        # Get the current time
        self.start_time = rospy.get_time()
        time_period = rospy.Duration(0.1) # seconds
        self.timer = rospy.Timer(time_period, self.pose_sim)

    def cbCmdVel(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def pose_sim(self, event):
        # Get time difference
        duration = rospy.get_time() - self.start_time
        
        # Convert the duration to a float value (in seconds)
        dt = duration

        #Calculate angle and wrap to 2pi
        self.angle += self.angular_speed * dt
        self.angle = self.angle % (2 * np.pi)
        
        x_dot = self.linear_speed * np.cos(self.angle)
        y_dot = self.linear_speed * np.sin(self.angle)
        theta_dot = self.angular_speed

        # Calculate position in x and y
        self.positionx += x_dot * dt
        self.positiony += y_dot * dt

        # Calculate the wheels' individual velocities
        if self.linear_speed != 0. or self.angular_speed != 0:
            self.wl = ((2 * self.linear_speed) - (self.angular_speed * self.l)) / (2 * self.r)
            self.wr = ((2 * self.linear_speed) + (self.angular_speed * self.l)) / (2 * self.r)
        else: 
            self.wl = 0.
            self.wr = 0.
        
        # Set and publish message 
        self.position.position.x = self.positionx
        self.position.position.y = self.positiony
        self.position.orientation.z = self.angle
        self.pose_pub.publish(self.position)
        self.wl_pub.publish(Float32(data=self.wl))
        self.wr_pub.publish(Float32(data=self.wr))
        self.start_time = rospy.get_time()

def main(args=None):
    pose_sim = Pose_Sim()
    rospy.spin()

if __name__ == "__main__":
    main()
