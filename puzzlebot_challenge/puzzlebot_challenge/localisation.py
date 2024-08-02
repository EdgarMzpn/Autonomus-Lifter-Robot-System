#!/usr/bin/env python
import rclpy
import numpy as np
from math import sin, cos, inf, pi
from numpy.linalg import inv
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from puzzlebot_msgs.msg import LandmarkList, Landmark, ArucoArray
from rclpy.qos import qos_profile_sensor_data

class Localisation(Node):
    """
    A ROS2 node for localizing a robot using odometry and a Kalman filter.

    This class subscribes to wheel velocities and ArUco marker information,
    estimates the robot's pose, and publishes the odometry.
    """
    def __init__(self):
        super().__init__('Odometry')

        # Initialize wheel variables
        self.wr = 0.0               # Right Wheel
        self.wl = 0.0               # Left Wheel
        self.linear_speed = 0.0     # Linear Speed
        self.angular_speed = 0.0    # Angular Speed
        self.l = 0.17               # Wheelbase
        self.r = 0.066              # Radius of the Wheel
        
        # Gains for model error
        self.kr = 0.15  
        self.kl = 0.15  

        # Starting pose for the puzzlebot
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0

        # Known landmarks for Kalman Filter
        self.landmark_true = {"1":[2.15, 3.21], "8": [3.14, 2.6065], "3": [3.14, 2.17], "7":[1.72, 1.1]}

        # Initialize varibles for measured landmarks
        self.measured_landmarks = LandmarkList()
        self.measured_landmarks.landmarks = []
        self.cube_id = '2'

        # Initialize subscribers for wheel velocities and markers information
        self.sub_wl = self.create_subscription(Float32, '/VelocityEncL', self.cbWl, qos_profile_sensor_data)
        self.sub_wr = self.create_subscription(Float32, '/VelocityEncR', self.cbWr, qos_profile_sensor_data)
        self.aruco_sub = self.create_subscription(ArucoArray, '/aruco_info', self.aruco_callback, 10)

        # Initialize publishers for odometry 
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        # Use timer to run the node and set the frequency rate
        self.start_time = self.get_clock().now()
        time_period = 0.02
        self.timer = self.create_timer(time_period, self.odom_reading)

    def cbWr(self, msg):
        """
        Callback function for the right wheel velocity subscriber.

        Args:
            msg (Float32): The message containing the right wheel velocity.
        """
        self.wr = msg.data

    def cbWl(self, msg):
        """
        Callback function for the left wheel velocity subscriber.

        Args:
            msg (Float32): The message containing the left wheel velocity.
        """
        self.wl = msg.data

    def aruco_callback(self, msg):
        """
        Callback function for the ArUco marker information subscriber.

        Args:
            msg (ArucoArray): The message containing the ArUco marker information.
        """
        self.aruco_info = msg
        self.measured_landmarks.landmarks = [] # Empty measured landmarks array
        if self.aruco_info.length != 0:
            for aruco in self.aruco_info.aruco_array:
                # If marker id in landmarks, save marker info
                if aruco.id != self.cube_id and aruco.id in np.array(['1', '8', '3', '7']): 
                    landmark = Landmark()
                    landmark.id = aruco.id
                    landmark.x, landmark.y = self.transform_cube_position(aruco.point.point)
                    self.measured_landmarks.landmarks.append(landmark)

    def transform_cube_position(self, aruco_point):
        """
        Transforms the position of a cube detected by the ArUco marker into world coordinates.

        Args:
            aruco_point (Point): The position of the ArUco marker in puzzlebot coordinates.

        Returns:
            tuple: The (x, y) position of the ArUco marker in world coordinates.
        """
        rotation_matrix = np.array([
            [cos(self.angle), -sin(self.angle)],
            [sin(self.angle), cos(self.angle)]
        ])

        # Change coordinates frame
        puzzlebot_coords = np.array([aruco_point.z, aruco_point.x])
        world_coords = rotation_matrix.dot(puzzlebot_coords)

        aruco_x = self.positionx + world_coords[0]
        aruco_y = self.positiony + world_coords[1]

        return aruco_x, aruco_y

    def kalman_filter(self):
        """
        Performs a Kalman filter update based on the measured landmarks.

        Returns:
            ndarray: The updated pose of the robot.
        """

        u_estimation = np.array([[self.positionx],[self.positiony],[self.angle]])

        # Position differential of landmark from current position estimation
        x_diff = self.landmark_true[self.measured_landmarks.landmarks[0].id][0] - self.positionx
        y_diff = self.landmark_true[self.measured_landmarks.landmarks[0].id][1] - self.positiony

        # Observation model estimation
        z_estimation = np.array([[np.sqrt(x_diff**2 + y_diff**2)],
                                 [np.arctan2(y_diff, x_diff)-self.angle]])
        # Observation model error matrix from measurement errors of 0.01 m
        R_k = np.array([[0.01, 0.0],
                        [0.0, 0.01]]) 

        # Linearized observation model
        G_k = np.array([    [-x_diff/np.sqrt(z_estimation[0][0]),     -y_diff/np.sqrt(z_estimation[0][0]),   0],
                            [y_diff/z_estimation[0][0],                -x_diff/z_estimation[0][0],             -1]])
        
        # Covariance of observation model
        Z_covariance = G_k.dot(self.sigma_estimation).dot(G_k.T) + R_k
        
        # Compute Kalman gain
        K_k = (self.sigma_estimation).dot(G_k.T).dot(inv(Z_covariance))

        # Compute polar coordinates for recorded landmark
        x_diff_ideal = self.measured_landmarks.landmarks[0].x - self.positionx
        y_diff_ideal = self.measured_landmarks.landmarks[0].y - self.positiony
        Z_ideal = np.array([[np.sqrt(x_diff_ideal**2 + y_diff_ideal**2)],
                            [np.arctan2(y_diff_ideal, x_diff_ideal)-self.angle]])

        # Adjust true position
        u_true = u_estimation + K_k.dot(Z_ideal-z_estimation)

        # Adjust covariance 
        self.sigma = (np.diag([1,1,1])-K_k.dot(G_k)).dot(self.sigma_estimation)

        return u_true

    def odom_reading(self):
        """
        Computes the odometry reading and updates the robot's pose.

        Uses the Kalman filter to refine the pose estimation when available and publishes the updated odometry.
        """

        # Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9
        
        self.linear_speed = self.r * (self.wr + self.wl) / 2.
        self.angular_speed = self.r * (self.wr - self.wl) / self.l

        # If not defined, create sigma variable
        if not hasattr(self, 'sigma'):
            self.sigma = np.zeros((3,3))

        # Define linearized robot model
        H_k = np.array([
            [1, 0, -self.dt * self.linear_speed * np.sin(self.angle)],
            [0, 1, self.dt * self.linear_speed * np.cos(self.angle)],
            [0, 0, 1]
        ])

        # Define Gaussian distribution matrix considering the wheels velocities
        Gaussian = np.array([[self.kr * abs(self.wr),   0],
                             [0,                        self.kl * abs(self.wl)]])

        # Define Taylor expansion series matrix 
        Taylor = np.diag([1/2 * self.r * self.dt, 1/2 * self.r * self.dt, 1/2 * self.r * self.dt]).dot(np.array([[np.cos(self.angle), np.cos(self.angle)],
                                                                                                                [np.sin(self.angle), np.sin(self.angle)],
                                                                                                                [2/self.l, -2/self.l]]))
        
        # Define noise covariance matrix
        Q_k = Taylor.dot(Gaussian).dot(Taylor.T)

        # Define covariance for robot model
        self.sigma_estimation = H_k.dot(self.sigma).dot(H_k.T) + Q_k

        # Compute estimation for position and orientation 
        self.angle += self.angular_speed * self.dt
        if np.fabs(self.angle) > np.pi:
            self.angle = self.angle - (2*np.pi*self.angle) / (np.fabs(self.angle))
        self.positionx += self.linear_speed * np.cos(self.angle) * self.dt
        self.positiony += self.linear_speed * np.sin(self.angle) * self.dt

        # Kalman filter in case of landmarks measured
        if len(self.measured_landmarks.landmarks) > 0:

            u_true = self.kalman_filter()

            self.positionx = u_true[0][0]
            self.positiony = u_true[1][0]
            self.angle = u_true[2][0]
        else: 
            self.sigma = self.sigma_estimation

        sigma_full = np.zeros((6, 6))
        sigma_full[:3, :3] = self.sigma  # Fill in the 3x3 position covariance
        sigma_full[3, 3] = 0.0001  # Small value for orientation around x (roll)
        sigma_full[4, 4] = 0.0001  # Small value for orientation around y (pitch)
        sigma_full[5, 5] = 0.0001  # Small value for orientation around z (yaw)

        # Publish odometry via odom topic
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"  # Set the frame id to "odom"
        odom.child_frame_id = "base_link"  # Set the child frame id to "base_link"
        odom.pose.pose.position.x = self.positionx
        odom.pose.pose.position.y = self.positiony  
        q = quaternion_from_euler(0., 0., self.angle)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.linear_speed
        odom.twist.twist.angular.z = self.angular_speed
        odom.pose.covariance = sigma_full.flatten().tolist()  # Set the pose covariance matrix as a list
        self.odom_pub.publish(odom) 
        self.start_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    odometry = Localisation()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()