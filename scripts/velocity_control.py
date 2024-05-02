#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class VelocityControl(object):
    def __init__(self):
        rospy.init_node('Velocity_Control')
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

        self.output_error = Point()

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.error_pub = rospy.Publisher('error', Point, queue_size=1)

        # Subscribers
        rospy.Subscriber('ideal', Pose, self.cbPose)
        rospy.Subscriber('odom', Odometry, self.cbOdom)

        # Start the timer now
        self.start_time = rospy.Time.now()
        time_period = 0.1
        self.timer = rospy.Timer(rospy.Duration(time_period), self.velocity_control)

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

    def PID_Linear(self, error, prev_error):
        # Proportional term
        P = self.linear_kp * error

        # Integral term
        self.linear_integral += error
        I = self.linear_ki * self.linear_integral

        # Derivative term
        derivative = error - prev_error
        D = self.linear_kd * derivative

        # Compute PID output
        output = P + I + D

        return output, error

    def PID_Angular(self, error, prev_error):
        # Proportional term
        P = self.angular_kp * error

        # Integral term
        self.angular_integral += error
        I = self.angular_ki * self.angular_integral

        # Derivative term
        derivative = error - prev_error
        D = self.angular_kd * derivative

        # Compute PID output
        output = P + I + D

        return output, error

    def resultant_error(self):
        x_error = self.desired_position_x - self.current_position_x
        y_error = self.desired_position_y - self.current_position_y

        self.total_position_error = np.sqrt(x_error**2 + y_error**2)

        desired_angle = np.arctan2(y_error, x_error)
        self.angle_error = desired_angle - self.current_angle


    def velocity_control(self, event):
        # Get time difference
        current_time = rospy.Time.now()
        duration = current_time - self.start_time
        self.start_time = current_time  # Update start time for the next iteration

        # Convert the duration to a float value (in seconds)
        dt = duration.to_sec()

        # Calculate resultant error
        self.resultant_error()

        # Adjust current pose
        self.output_position, self.prev_position_error = self.PID_Linear(self.total_position_error, self.prev_position_error)
        self.output_angle, self.prev_angle_error = self.PID_Angular(self.angle_error, self.prev_angle_error)

        if abs(self.prev_angle_error) > 0.1:
            self.output_velocity.angular.z = self.output_angle
            self.output_velocity.linear.x = 0.0
        else:
            self.output_velocity.linear.x = self.output_position
            self.output_velocity.angular.z = 0.0

        self.output_error.x = self.total_position_error
        self.output_error.y = self.prev_angle_error

        self.cmd_vel_pub.publish(self.output_velocity)
        self.error_pub.publish(self.output_error)

def main():
    velocity = VelocityControl()
    rospy.spin()

if __name__ == "__main__":
    main()
