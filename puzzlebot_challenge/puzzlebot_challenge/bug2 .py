#!/usr/bin/env python3

import rclpy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import atan2, pi, sqrt

class Bug2Node():
    def __init__(self, target, position):
        self.xt = target[0]
        self.yt = target[1]
        self.xk = position[0]
        self.yk = position[1]
        self.thetak = position[2]
        self.reached_target = False
        self.state = "GO_TO_GOAL"
        self.hit_point = None
        self.leave_point = None

        # Initialize PID controller parameters
        self.linear_kp = 0.15
        self.linear_ki = 0.0
        self.linear_kd = 0.05
        self.linear_integral = 0.0

        self.angular_kp = 0.54
        self.angular_ki = 0.0
        self.angular_kd = 0.15
        self.angular_integral = 0.0

        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0

        self.total_position_error = 0.0
        self.angle_error = 0.0

    def PID_Linear(self, error, prev_error):
        P = self.linear_kp * error
        self.linear_integral += error
        I = self.linear_ki * self.linear_integral
        derivative = error - prev_error
        D = self.linear_kd * derivative
        output = P + I + D
        return output

    def PID_Angular(self, error, prev_error):
        P = self.angular_kp * error
        self.angular_integral += error
        I = self.angular_ki * self.angular_integral
        derivative = error - prev_error
        D = self.angular_kd * derivative
        output = P + I + D
        return output
        
    def resultant_error(self):
        x_error = self.xt - self.xk
        y_error = self.yt - self.yk
        self.total_position_error = np.sqrt(x_error**2 + y_error**2)
        desired_angle = np.arctan2(y_error, x_error)
        self.angle_error = desired_angle - self.thetak
    

    def scan_callback(self, data):
        if self.reached_target:
            self.stop_robot()
            return
        
        min_dist = min(data.ranges)
        min_dist_idx = data.ranges.index(min_dist)
        angle_to_obstacle = data.angle_min + min_dist_idx * data.angle_increment
        angle_to_target = atan2(self.yt - self.yk, self.xt - self.xk)
        distance_to_target = sqrt((self.xt - self.xk)**2 + (self.yt - self.yk)**2)

        if distance_to_target < 0.1:  # Threshold to consider target reached
            self.reached_target = True
            print("Target reached")
            self.stop_robot()
            return

        if self.state == "GO_TO_GOAL":
            if min_dist < 0.3:
                self.state = "FOLLOW_WALL"
                self.hit_point = (self.xk, self.yk)
                print("Hit Point: ", self.hit_point)
            else:
                self.move_towards_goal(angle_to_target)
                # print("Go to Goal")
        elif self.state == "FOLLOW_WALL":
            leave_distance = self.distance_to_line(self.xk, self.yk, self.xt, self.yt)
            if min_dist >= 0.3 and leave_distance < 0.05:
                self.state = "GO_TO_GOAL"
                self.leave_point = (self.xk, self.yk)
                print("Leave Point: ", self.leave_point)
            else:
                self.follow_wall(angle_to_obstacle, min_dist)
                # print("Follow Wall")

    def move_towards_goal(self, angle_to_target):
        vel_msg = Twist()
        self.resultant_error()
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
        orientation_from_wall = angle_to_obstacle + pi/2
        vel_msg = Twist()
        print(min_dist)
        if abs(0.15 - min_dist) > 0.05 and abs(orientation_from_wall) < 0.25:
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = (0.5 * (0.15-min_dist))
            print("off distance")
        elif abs(orientation_from_wall) > 0.15:
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = (1.0 * orientation_from_wall + (0.3 * (0.15-min_dist))) 
            print("off orientation")
        else:
            vel_msg.linear.x = 0.2 * min_dist
            vel_msg.angular.z = (0.9 * orientation_from_wall + (0.3 * (0.15-min_dist)))
            print("good")
        self.cmd_vel_pub.publish(vel_msg)

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)

    def distance_to_line(self, x, y, xt, yt):
        # Calculate the perpendicular distance from (x, y) to the line connecting the start and goal (0,0) to (xt, yt)
        num = abs((yt - 0) * x - (xt - 0) * y)
        den = sqrt(yt**2 + xt**2)
        return num / den


if __name__ == '__main__':
    try:
        xt = 1.45
        yt = 1.2
        bug2_node = Bug2Node(xt, yt)
        bug2_node.run()
    except rospy.ROSInterruptException:
        pass
