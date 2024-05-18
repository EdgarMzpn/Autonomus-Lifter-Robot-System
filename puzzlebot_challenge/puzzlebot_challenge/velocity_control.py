#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class Velocity_Control(Node):
    def __init__(self):
        super().__init__('Velocity_Control')
        # Initialize puzzlebot variables
        self.desired_position_x = 0.0
        self.desired_position_y = 0.0
        self.desired_angle = 0.0

        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_angle = 0.0

        self.total_position_error = 0.0
        self.angle_error = 0.0

        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0

        self.output_position = 0.0
        self.output_angle = 0.0
        self.output_velocity = Twist()

        # Initialize PID controller parameters
        self.linear_kp = 0.15
        self.linear_ki = 0.0
        self.linear_kd = 0.05
        self.linear_integral = 0.0

        self.angular_kp = 0.48
        self.angular_ki = 0.0
        self.angular_kd = 0.02
        self.angular_integral = 0.0

        self.integral = 0.0

        self.output_error = Point()
        self.arrive = Bool()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.error_pub = self.create_publisher(Point, 'error', 1)
        self.arrive_pub = self.create_publisher(Bool, 'arrive', 1)

        # Subscribers
        self.pose_sub = self.create_subscription(Pose, 'pose_ideal', self.cbPose, 10)
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
        quaternion = msg.pose.pose.orientation
        self.current_angle = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]
    
    def PID_General(self, error, prev_error, kp, ki, kd):
        # Proportional term
        P = kp * error
        
        # Integral term
        self.integral += error
        I = ki * self.integral
        
        # Derivative term
        derivative = error - prev_error
        D = kd * derivative
        
        # Compute PID output
        output = P + I + D
        
        return output, error

    def resultant_error(self):
        x_error = self.desired_position_x - self.current_position_x
        y_error = self.desired_position_y - self.current_position_y

        self.total_position_error = np.sqrt(x_error**2 + y_error**2)

        desired_angle = np.arctan2(y_error, x_error)
        self.angle_error = desired_angle - self.current_angle

    def velocity_control(self):
        #Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9

        # Calculate resultant error
        self.resultant_error()

        # Adjust current pose
        self.output_position, self.prev_position_error = self.PID_General(self.total_position_error, self.prev_position_error, self.linear_kp, self.linear_ki, self.linear_kd)
        self.output_angle, self.prev_angle_error = self.PID_General(self.angle_error, self.prev_angle_error, self.angular_kp, self.angular_ki, self.angular_kd)
        
        # if abs(self.prev_angle_error) > 0.1:
        self.output_velocity.angular.z = self.output_angle
            # self.output_velocity.linear.x = 0.0
        # else:
        self.output_velocity.linear.x = self.output_position
            # self.output_velocity.angular.z = 0.0

        self.output_error.x = self.total_position_error
        self.output_error.y = self.prev_angle_error

        self.cmd_vel_pub.publish(self.output_velocity)
        self.error_pub.publish(self.output_error)

        if self.total_position_error < 0.5 and self.total_position_error > -0.5:
            self.arrive.data = True
        else:
            self.arrive.data = False

        self.arrive_pub.publish(self.arrive)
        

def main(args=None):
    rclpy.init(args=args)
    velocity = Velocity_Control()
    rclpy.spin(velocity)
    velocity.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        