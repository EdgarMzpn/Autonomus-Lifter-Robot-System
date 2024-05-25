#!/usr/bin/env python3
import rospy
import math
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import enum

# Define the state machine for the Bug2 algorithm
class StateMachine(enum.Enum):
    LOOK_TOGOAL = 1
    FOLLOW_LINE = 2
    WALL_FOLLOW = 3
    STOP = 4

class Bug2Controller:
    def __init__(self):
        # Initialize various parameters and ROS node
        self.yaw = 0.0
        self.current_state = StateMachine.LOOK_TOGOAL

        # Initialize current pose of the robot
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.current_pose.pose.position.x = 2
        self.current_pose.pose.position.y = 2

        # Initialize starting pose of the robot
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = "world"
        self.start_pose.pose.position.x = 2
        self.start_pose.pose.position.y = 2

        # Set the goal position for the robot
        self.goal = PoseStamped()
        self.goal.header.frame_id = "world"
        self.goal.pose.position.x = 0.0
        self.goal.pose.position.y = -2.45

        self.cmd_vel = Twist()  # Velocity command
        self.hitpoint = None  # Point where the robot hits an obstacle
        self.distance_moved = 0.0  # Distance moved by the robot

        # Distances to the nearest obstacles in different directions
        self.front_distance = 0.0
        self.frontL_distance = 0.0
        self.left_distance = 0.0

        # Calculate line parameters (slope and y-intercept) from start to goal
        self.line_slope_m = (self.goal.pose.position.y - self.start_pose.pose.position.y) / (
                self.goal.pose.position.x - self.start_pose.pose.position.x)
        self.line_slope_b = self.start_pose.pose.position.y - (self.line_slope_m * self.start_pose.pose.position.x)

        # Initialize ROS publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)

        # Initialize ROS subscribers for odometry, goal, and laser scan data
        rospy.Subscriber('/puzzlebot_1/base_pose_ground_truth', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/puzzlebot_1/scan', LaserScan, self.scan_callback)

        # Set up shutdown behavior
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(50)  # Loop rate in Hz

    def wrap_to_pi(self, angle):
        # Wrap an angle to the range [-pi, pi]
        if np.fabs(angle) > np.pi:
            angle = angle - (2*np.pi*angle) / (np.fabs(angle))
        return angle

    def look_to_goal(self):
        # Orient the robot towards the goal
        quaternion = (self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]  # Get the current yaw of the robot

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(self.goal.pose.position.y - self.current_pose.pose.position.y, self.goal.pose.position.x - self.current_pose.pose.position.x)
        angle_error = self.wrap_to_pi(angle_to_goal - self.yaw)

        # Rotate the robot towards the goal if the angle error is significant
        if np.fabs(angle_error) > np.pi/90:
            self.cmd_vel.angular.z = 0.5 if angle_error > 0 else -0.5
        else:
            self.cmd_vel.angular.z = 0.0
            self.current_state = StateMachine.FOLLOW_LINE  # Switch to FOLLOW_LINE state

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def move_to_goal(self):
        # Move the robot towards the goal
        if np.any((self.front_distance < 0.3)):  # Stop if an obstacle is detected in front
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.hitpoint = self.current_pose.pose.position  # Record the hitpoint
            self.current_state = StateMachine.WALL_FOLLOW  # Switch to WALL_FOLLOW state
        else:
            self.cmd_vel.linear.x = 0.5
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def follow_wall(self):
        # Follow the wall until a certain condition is met
        closestGoalLine_x = self.current_pose.pose.position.x
        closestGoalLine_y = self.line_slope_m * self.current_pose.pose.position.x + self.line_slope_b

        self.distance_moved = math.sqrt((self.current_pose.pose.position.x - self.hitpoint.x)**2 + (self.current_pose.pose.position.y - self.hitpoint.y)**2)
        distance_to_line = math.sqrt((closestGoalLine_x - self.current_pose.pose.position.x)**2 + (closestGoalLine_y - self.current_pose.pose.position.y)**2)

        if distance_to_line < 0.1 and self.distance_moved > 0.5:
            distance_to_goal = math.sqrt((self.goal.pose.position.x - self.current_pose.pose.position.x)**2 + (self.goal.pose.position.y - self.current_pose.pose.position.y)**2)
            hitpoint_distance_to_goal = math.sqrt((self.goal.pose.position.x - self.hitpoint.x)**2 + (self.goal.pose.position.y - self.hitpoint.y)**2)

            if hitpoint_distance_to_goal > distance_to_goal:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.current_state = StateMachine.LOOK_TOGOAL  # Switch to LOOK_TOGOAL state
                return
        elif np.any((self.front_distance < 0.3)):
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = -0.3
        elif np.any((self.frontL_distance >= 0.2)):
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = 0.3
        else:
            self.cmd_vel.linear.x = 0.3
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def goal_callback(self, msg):
        # Update the goal when a new goal message is received
        self.goal = msg
        self.start_pose = self.current_pose

        self.line_slope_m = (self.goal.pose.position.y - self.start_pose.pose.position.y) / (
                    self.goal.pose.position.x - self.start_pose.pose.position.x)
        self.line_slope_b = self.start_pose.pose.position.y - (self.line_slope_m * self.start_pose.pose.position.x)
        self.current_state = StateMachine.LOOK_TOGOAL  # Switch to LOOK_TOGOAL state

    def odom_callback(self, msg):
        # Update the current pose based on odometry data
        self.current_pose.pose = msg.pose.pose

    def scan_callback(self, msg):
        # Update the distances to obstacles based on laser scan data
        data = np.array(msg.ranges)
        self.front_distance = np.min(data[141:220])
        self.frontL_distance = np.min(data[221:310])

    def stop(self):
        # Stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        # Main loop to control the robot
        while not rospy.is_shutdown():        
            if self.current_state is StateMachine.LOOK_TOGOAL:
                self.look_to_goal()
            elif self.current_state is StateMachine.FOLLOW_LINE:
                self.move_to_goal()
            elif self.current_state is StateMachine.WALL_FOLLOW:
                self.follow_wall()
            elif self.current_state is StateMachine.STOP:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                print("Found goal!")

            self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command
            goal_distance = math.sqrt((self.goal.pose.position.x - self.current_pose.pose.position.x)**2 + (self.goal.pose.position.y - self.current_pose.pose.position.y)**2)

            if goal_distance < 0.15:  # Stop if the goal is reached
                self.current_state = StateMachine.STOP

            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('controller_node', anonymous=True)
        controller = Bug2Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass