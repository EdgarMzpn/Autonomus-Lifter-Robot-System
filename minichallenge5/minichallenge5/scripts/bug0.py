#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import atan2, pi, cos, sin

class Bug0Node():
    def __init__(self, xt, yt):
        rospy.init_node('bug0_node', anonymous=True)
        self.scan_sub = rospy.Subscriber('/puzzlebot_1/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=10)
        self.xt = xt
        self.yt = yt
        self.xk = 0.0
        self.yk = 0.0
        self.thetak = 0.0
        self.reached_target = False
        self.state = "GO_TO_GOAL"

         # Initialize PID controller parameters
        self.linear_kp = 0.15
        self.linear_ki = 0.0
        self.linear_kd = 0.05
        self.linear_integral = 0.0

        self.angular_kp = 0.54#0.48
        self.angular_ki = 0.0
        self.angular_kd = 0.15#0.02
        self.angular_integral = 0.0

        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0

        self.total_position_error = 0.0
        self.angle_error = 0.0

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

        return output

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

        return output
        
    def resultant_error(self):
        x_error = self.xt - self.xk
        y_error = self.yt - self.yk

        self.total_position_error = np.sqrt(x_error**2 + y_error**2)

        desired_angle = np.arctan2(y_error, x_error)
        self.angle_error = desired_angle - self.thetak
    
    def odom_callback(self, msg):
        self.xk = msg.pose.pose.position.x
        self.yk = msg.pose.pose.position.y

        # Get current angle from quaternion
        quaternion = msg.pose.pose.orientation
        self.thetak = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

    def scan_callback(self, data):
        if self.reached_target:
            self.stop_robot()
            return

        min_dist = min(data.ranges)
        min_dist_idx = data.ranges.index(min_dist)
        angle_to_obstacle = data.angle_min + min_dist_idx * data.angle_increment
        angle_to_target = atan2(self.yt, self.xt)

        distance_to_target = (self.xt*2 + self.yt*2)*0.5

        # print(angle_to_target)
        # print(distance_to_target)

        if distance_to_target < 0.1:  # Threshold to consider target reached
            self.reached_target = True
            self.stop_robot()
            return

        if self.state == "GO_TO_GOAL":
            if min_dist < 0.3:
                self.state = "FOLLOW_WALL"
                print("Follow Wall")
            else:
                self.move_towards_goal(angle_to_target)
                print("Go to Goal")
        elif self.state == "FOLLOW_WALL":
            if min_dist >= 0.3:
                self.state = "GO_TO_GOAL"
                print("Go to goal")
            else:
                self.follow_wall(angle_to_obstacle, min_dist)
                print("Follow wall")


    def move_towards_goal(self, angle_to_target):
        vel_msg = Twist()
        self.resultant_error()
        # print(self.prev_angle_error)
        if abs(self.prev_angle_error) > 0.2:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = self.PID_Angular(self.angle_error, self.prev_angle_error)
        else:
            vel_msg.linear.x = self.PID_Linear(self.total_position_error, self.prev_position_error)
            vel_msg.angular.z = 0.0
        self.prev_position_error = self.total_position_error
        self.prev_angle_error = self.angle_error
        self.cmd_vel_pub.publish(vel_msg)

    def follow_wall(self, angle_to_obstacle, min_dist):
        orientation_from_wall = (angle_to_obstacle + pi/2)
        vel_msg = Twist()
        if min_dist < 0.15 and abs(orientation_from_wall) > 0.1:
            vel_msg.linear.x = 0.0 #0.5 * min_dist
            vel_msg.angular.z = 0.6* orientation_from_wall
        else:
            vel_msg.linear.x = 0.15 * min_dist
            vel_msg.angular.z = 0.8* orientation_from_wall
            
        print("Angle to obstacle: "+str(orientation_from_wall))
        self.cmd_vel_pub.publish(vel_msg)

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        xt = 1.45
        yt = 1.2
        bug0_node = Bug0Node(xt, yt)
        bug0_node.run()
    except rospy.ROSInterruptException:
        pass