#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose
from std_srvs.srv import Empty

class Pose_Sim(Node):

    def __init__(self):
        super().__init__('Pose_Sim')
        # Initialize wheel variables
        self.wl = 0.0
        self.wr = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.l = 0.19
        self.r = 0.05

        # Starting pose for the puzzlebot
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0
        self.position = Pose()

        self.lastTime = 0.0
        self.dt = 0.0
        
        # Define the Publishers
        self.wl_pub = self.create_publisher(Float32, "wl", 1)
        self.wr_pub = self.create_publisher(Float32, "wr", 1)
        self.pose_pub = self.create_publisher(Pose, 'pose', 1)

        # Define the Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cbCmdVel, 10)

        # Start timer
        self.start_time = self.get_clock().now()
        self.get_logger().info('Testing process')

    def cbCmdVel(self, msg):
        self.linear_speed = msg.linear
        self.angular_speed = msg.angular

    def pose_sim(self):
        # if gazebo sim is restarted
        if self.start_time <= 1:
            self.angle = -1.57
            self.positionx = -0.95
            self.positiony = 0.75
            
        #Get time difference
        dt = self.start_time - lastTime

        x_dot = self.linear_speed * np.cos(self.angle)
        y_dot = self.linear_speed * np.sin(self.angle)
        theta_dot = self.angular_speed

        # Calculate the wheels' individual velocities

        self.wl = ((2 * self.linear_speed) - (self.angular_speed + self.l)) / (2 * self.r)
        self.wr = (2 * self.linear_speed / self.r) - self.wl


        
        #Calculate angle and wrap to 2pi
        self.angle += 0.05*((self.wr - self.wl) / 0.19) * dt
        self.angle = self.angle % 6.28
        

        # Calculate position in x and y
        self.positionx += 0.05*((self.wr + self.wl) / 2) * dt * np.cos(self.angle)
        self.positiony += 0.05*((self.wr + self.wl) / 2) * dt * np.sin(self.angle)
        
        #set and publish message 
        self.position.position.x = self.positionx
        self.position.position.y = self.positiony
        self.position.orientation.z = self.angle
        self.pose_pub.publish(self.position)
        self.wl_pub.publish(self.wl)
        self.wr_pub.publish(self.wr)
        
        print(self.position)
        lastTime = self.start_time

def main(args=None):
    rclpy.init(args=args)
    pose_sim = Pose_Sim()
    rclpy.spin(pose_sim)
    pose_sim.destroy_node()
    rclpy.shutdown

if __name__ == "__main__":
    main()