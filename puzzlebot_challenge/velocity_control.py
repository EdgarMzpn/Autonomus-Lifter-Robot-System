#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class Velocity_Control(Node):
    def __init__(self):
        super().__init__('Velocity Control')
        # Initialize puzzlebot variables
        self.desired_position_x = 0.0
        self.desired_position_y = 0.0
        self.desired_angle = 0.0

        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_angle = 0.0

        self.total_position_error = 0.0
        self.angle_error = 0.0

        self.output_position = 0.0
        self.output_angle = 0.0
        self.output_velocity = Twist()

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
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.cbOdom, 10)

        # Start the timer now
        self.start_time = self.get_clock().now()
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.velocity_control)

    def cbPose(self, msg):
        self.desired_position_x = msg.position.x
        self.desired_position_y = msg.position.y
        self.desired_angle = msg.orientation.z

    def cbOdom(self, msg):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y

        # Get current angle from quaternion
        quaternion = self.odometry.pose.pose.orientation
        self.current_angle = euler_from_quaternion(quaternion)[2]

    def PID(self, error):
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

    def resultant_error(self):
        x_error = self.current_position_x - self.position_x
        y_error = self.current_position_y - self.position_y

        self.total_position_error = np.sqrt(error_x, error_y)

        self.angle_error = np.arctan2(error_y, error_x)

    def velocity_control(self):
        #Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9

        # Calculate resultant error
        resultant_error()

        # Adjust current pose
        self.output_position = Velocity_Control.PID(self.total_position_error)
        self.output_angle = Velocity_Control.PID(self.angle_error)

        self.output_velocity.linear.x = output_position
        self.output_velocity.angular.z = output_angle

        cmd_vel_pub.publish(output_velocity)

def main(args=None):
    rclpy.init(args=args)
    velocity = Velocity_Control()
    rclpy.spin(velocity)
    velocity.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        