#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from tf_transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import enum

# Define the state machine for the Bug2 algorithm
class StateMachine(enum.Enum):
    """
    Enum to define the states for the Bug2 algorithm.
    
    States:
        LOOK_TOGOAL: Orient the robot towards the goal.
        FOLLOW_LINE: Move the robot towards the goal.
        FOLLOW_WALL: Follow the wall when an obstacle is encountered.
        STOP: Stop the robot when the goal is reached.
    """
    LOOK_TOGOAL = 1
    FOLLOW_LINE = 2
    FOLLOW_WALL = 3
    STOP = 4

class Bug2Controller(Node):
    """
    A ROS2 node implementing the Bug2 algorithm for navigation.

    The Bug2 algorithm navigates a robot towards a goal while avoiding obstacles.
    """
    def __init__(self):
        """
        Initializes the Bug2Controller node.
        """
        super().__init__('Bug2')
        # Initialize various parameters and ROS node
        self.yaw = 0.0
        self.current_state = StateMachine.LOOK_TOGOAL
        self.arrived = False
        # Initialize current pose of the robot
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.current_pose.pose.position.x = 0.
        self.current_pose.pose.position.y = 0.

        # Initialize starting pose of the robot
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = "world"
        self.start_pose.pose.position.x = 0.
        self.start_pose.pose.position.y = 0.

        # Set the goal position for the robot
        self.goal = PoseStamped()
        self.goal.header.frame_id = "world"
        self.goal.pose.position.x = 0.0
        self.goal.pose.position.y = 0.0

        self.cmd_vel = Twist()  # Velocity command
        self.hitpoint = None  # Point where the robot hits an obstacle
        self.distance_moved = 0.0  # Distance moved by the robot

        # Distances to the nearest obstacles in different directions
        self.front_distance = 1.0
        self.frontL_distance = 0.0
        self.left_distance = 0.0

        # Initialize publisher for velocity commands and completition state
        self.cmd_vel_pub = self.create_publisher( Twist, '/cmd_vel', 1)
        self.arrive_pub = self.create_publisher( Bool, '/arrived', 1)

        # Initialize subscribers for odometry, goal, laser scan data and node activation stream
        self.create_subscription( Odometry, '/odom', self.odom_callback, 1)
        self.create_subscription( PoseStamped, '/goal', self.goal_callback, 1)
        self.create_subscription( LaserScan, '/filtered_scan', self.scan_callback, 1)
        self.create_subscription( Bool, '/bug2_run', self.run, 1)

    def wrap_to_pi(self, angle):
        """
        Wraps an angle to the range [-pi, pi].

        Args:
            angle (float): The angle to be wrapped.

        Returns:
            float: The wrapped angle.
        """
        if np.fabs(angle) > np.pi:
            angle = angle - (2*np.pi*angle) / (np.fabs(angle))
        return angle

    def look_to_goal(self):
        """
        Orients the robot towards the goal.

        Calculates the angle to the goal and adjusts the robot's yaw.
        """
        quaternion = (self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]  # Get the current yaw of the robot

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(self.goal.pose.position.y - self.current_pose.pose.position.y, self.goal.pose.position.x - self.current_pose.pose.position.x)
        angle_error = self.wrap_to_pi(angle_to_goal - self.yaw)

        # Rotate the robot towards the goal if the angle error is significant
        if np.fabs(angle_error) > np.pi/180:
            self.cmd_vel.angular.z = 0.05 if angle_error > 0 else -0.05
        else:
            self.cmd_vel.angular.z = 0.0
            self.current_state = StateMachine.FOLLOW_LINE  # Switch to FOLLOW_LINE state

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def move_to_goal(self):
        """
        Moves the robot towards the goal.

        Stops and switches to wall following if an obstacle is detected.
        """
        if np.any((self.front_distance < 0.3)):  # Stop if an obstacle is detected in front
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.hitpoint = self.current_pose.pose.position  # Record the hitpoint
            self.current_state = StateMachine.FOLLOW_WALL  # Switch to FOLLOW_WALL state
        else:
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def follow_wall(self):
        """
        Follows the wall until a condition to return to goal seeking is met.

        Uses the distances to obstacles to control the robot's movements.
        """
        # Estimate closest point of the traced line from current position
        closestGoalLine_x = self.current_pose.pose.position.x
        closestGoalLine_y = self.line_slope_m * self.current_pose.pose.position.x + self.line_slope_b

        self.distance_moved = math.sqrt((self.current_pose.pose.position.x - self.hitpoint.x)**2 + (self.current_pose.pose.position.y - self.hitpoint.y)**2)
        distance_to_line = math.sqrt((closestGoalLine_x - self.current_pose.pose.position.x)**2 + (closestGoalLine_y - self.current_pose.pose.position.y)**2)

        # Check to follow line and quit following obstacle
        if distance_to_line < 0.15 and self.distance_moved > 0.3:
            distance_to_goal = math.sqrt((self.goal.pose.position.x - self.current_pose.pose.position.x)**2 + (self.goal.pose.position.y - self.current_pose.pose.position.y)**2)
            hitpoint_distance_to_goal = math.sqrt((self.goal.pose.position.x - self.hitpoint.x)**2 + (self.goal.pose.position.y - self.hitpoint.y)**2)

            if hitpoint_distance_to_goal > distance_to_goal:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.current_state = StateMachine.LOOK_TOGOAL  # Switch to LOOK_TOGOAL state
                return
        
        # Rotate robot in its axis
        elif np.any((self.front_distance < 0.3)):
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = -0.1
        # Steer towards obstacle
        elif np.any((self.frontL_distance >= 0.3)):
            self.cmd_vel.linear.x = 0.1
            self.cmd_vel.angular.z = 0.1
        # Steer away from obstacle
        elif np.any((self.frontL_distance < 0.25)):
            self.cmd_vel.linear.x = 0.08
            self.cmd_vel.angular.z = -0.05
        else:
            self.cmd_vel.linear.x = 0.1
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def goal_callback(self, msg):
        """
        Callback for the goal topic.

        Updates the goal and calculates the line equation from the start to the goal.

        Args:
            msg (PoseStamped): The new goal message.
        """
        self.goal = msg # Update goal
        self.start_pose = self.current_pose # Update starting position

        self.line_slope_m = (self.goal.pose.position.y - self.start_pose.pose.position.y) / (
                    self.goal.pose.position.x - self.start_pose.pose.position.x)    # Trace line to the objective
        self.line_slope_b = self.start_pose.pose.position.y - (self.line_slope_m * self.start_pose.pose.position.x)
        self.current_state = StateMachine.LOOK_TOGOAL  # Switch to LOOK_TOGOAL state

    def odom_callback(self, msg):
        """
        Callback for the odometry topic.

        Updates the current pose based on odometry data.

        Args:
            msg (Odometry): The odometry message.
        """
        self.current_pose.pose = msg.pose.pose

    def scan_callback(self, msg):
        """
        Callback for the laser scan topic.

        Updates the distances to obstacles based on laser scan data.

        Args:
            msg (LaserScan): The laser scan message.
        """
        data = np.array(msg.ranges)
        self.front_distance = np.min(np.concatenate((data[0:40], data[680:720]))) # Define range of indexes for the front of the robot
        self.frontL_distance = np.min(data[41:130]) # Define range of indexes for the left side of the robot

    def stop(self):
        """
        Stops the robot by setting its velocities to zero.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self, msg):
        """
        Main loop to control the robot based on the current state.

        Args:
            msg (Bool): Message to trigger the running of the Bug2 algorithm.
        """     
        if self.current_state is StateMachine.LOOK_TOGOAL:
            self.look_to_goal()
        elif self.current_state is StateMachine.FOLLOW_LINE:
            self.move_to_goal()
        elif self.current_state is StateMachine.FOLLOW_WALL:
            self.follow_wall()
        elif self.current_state is StateMachine.STOP:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.arrived = True
            print("Found goal!")

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command
        self.arrive_pub.publish(Bool(data = self.arrived)) # Publish completition state
        distance_to_goal = math.sqrt((self.goal.pose.position.x - self.current_pose.pose.position.x)**2 + (self.goal.pose.position.y - self.current_pose.pose.position.y)**2)

        if distance_to_goal < 0.15:  # Stop if the goal is reached
            self.current_state = StateMachine.STOP


def main(args=None):
    rclpy.init(args=args)
    bug = Bug2Controller()
    rclpy.spin(bug)
    bug.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
