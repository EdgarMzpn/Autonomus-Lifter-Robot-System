#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class Velocity_Control(Node):
    def __init__(self):
        super().__init__('Velocity Control')
        # Initialize puzzlebot variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.angle = 0.0

        self.output_x = 0.0
        self.output_y = 0.0
        self.output_angle = 0.0

        self.twist = Twist()
        self.odometry = Odometry()
        self.pose = Pose()

        # Initialize PID controller parameters
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        # Subscribers
        self.pose_sub = self.create_subscription(Pose, 'pose', self.cbPose, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.cdOdom, 10)

        # Start the timer now
        self.start_time = self.get_clock().now()
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.velocity_control)

    def cbPose(self, msg):
        self.actual_position_x = msg.position.x
        self.position_y = msg.position.y
        self.angle = msg.orientation.z

    def cbOdom(self, msg):
        self.odometry = self.msg

    def PID(self, value, setpoint):
        error = setpoint - actual_value
        
        # Proportional term
        P = self.kp * error
        
        # Integral term
        self.integral += error
        I = self.ki * self.integral
        
        # Derivative term
        derivative = error - self.prev_error
        D = self.kd * derivative
        
        # Compute PID output
        output = P + I + D
        
        # Update previous error
        self.prev_error = error
        
        return output

    def velocity_control(self):
        #Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9

        # Get current angle from quaternion
        quaternion = self.odometry.pose.pose.orientation
        current_angle = euler_from_quaternion(quaternion)[2]

        # Adjust current pose
        self.output_x = PID(self.odometry.pose.pose.position.x)
        self.output_y = PID(self.odometry.pose.pose.position.y)
        self.output_angle = PID(current_angle)

        twist
        