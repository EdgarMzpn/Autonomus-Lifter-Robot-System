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

        # Starting pose for the puzzlebot
        self.angle = -1.574
        self.positionx = -0.95
        self.positiony = 0.75
        self.position = Pose()

        self.lastTime = 0.0
        self.dt = 0.0
        
        # Define the Subscribers
        self.wl_sub = self.create_subscription(Float32, "wl", self.cbWL, 10)
        self.wr_sub = self.create_subscription(Float32, "wr", self.cbWR, 10)

        # Define the Publishers
        self.pose_pub = self.create_publisher(Pose, "pose", 1)

        # Start timer
        self.start_time = self.get_clock().now()

    def cbWL(self, msg):
        self.wl = msg.data

    def cbWR(self, msg):
        self.wr = msg.data

    def pose_sim(self):
        # if gazebo sim is restarted
        if self.start_time <= 1:
            angle = -1.57
            positionx = -0.95
            positiony = 0.75
            
        #Get time difference
        dt = self.start_time - lastTime
        
        #Calculate angle and wrap to 2pi
        angle += 0.05*((self.wr - self.wl) / 0.19) * dt
        angle = angle % 6.28
        
        #Calculate position in x and y
        positionx += 0.05*((self.wr + self.wl) / 2) * dt * np.cos(angle)
        positiony += 0.05*((self.wr + self.wl) / 2) * dt * np.sin(angle)
        
        #set and publish message 
        self.position.position.x = positionx
        self.position.position.y = positiony
        self.position.orientation.z = angle
        self.pose_pub.publish(self.position)
        
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