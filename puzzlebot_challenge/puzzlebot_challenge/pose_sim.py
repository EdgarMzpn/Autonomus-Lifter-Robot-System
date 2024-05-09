#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty

class Pose_Sim(Node):

    def __init__(self):
        super().__init__('Pose_Sim')
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
        self.wl_pub = self.create_publisher(Float32, "VelocityEncL", 1)
        self.wr_pub = self.create_publisher(Float32, "VelocityEncR", 1)
        self.pose_pub = self.create_publisher(Pose, 'pose', 1)

        # Define the Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cbCmdVel, 10)

        # Get the current time
        self.start_time = self.get_clock().now()
        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.pose_sim)

    def cbCmdVel(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def pose_sim(self):
        
        #Get time difference
        self.duration = self.get_clock().now() - self.start_time
        
        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9

        #Calculate angle and wrap to 2pi
        self.angle += self.angular_speed * self.dt
        self.angle = self.angle % 6.28
        
        x_dot = self.linear_speed * np.cos(self.angle)
        y_dot = self.linear_speed * np.sin(self.angle)
        theta_dot = self.angular_speed

        # Calculate position in x and y
        self.positionx += x_dot * self.dt
        self.positiony += y_dot * self.dt

        # Calculate the wheels' individual velocities
        if(self.linear_speed != 0. or self.angular_speed != 0):
            self.wl = ((2 * self.linear_speed) - (self.angular_speed * self.l)) / (2 * self.r)
            self.wr = ((2 * self.linear_speed) + (self.angular_speed * self.l)) / (2 * self.r)
        else: 
            self.wl = 0.
            self.wr = 0.
        
        
        #set and publish message 
        self.position.position.x = self.positionx
        self.position.position.y = self.positiony
        self.position.orientation.z = self.angle
        self.pose_pub.publish(self.position)
        self.wl_pub.publish(Float32(data = self.wl))
        self.wr_pub.publish(Float32(data = self.wr))
        self.start_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    pose_sim = Pose_Sim()
    rclpy.spin(pose_sim)
    pose_sim.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()