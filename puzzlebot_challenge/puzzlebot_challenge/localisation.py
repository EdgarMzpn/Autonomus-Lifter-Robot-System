#!/usr/bin/env python
import rclpy
import math
import numpy as np
from math import sin, cos, inf, pi
from numpy.linalg import inv
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf_transformations import quaternion_from_euler
from puzzlebot_msgs.msg import LandmarkList, Landmark, ArucoArray
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

class Localisation(Node):
    def __init__(self):
        super().__init__('Odometry')

        # Initialize wheel variables
        self.wr = 0.0               # Right Wheel
        self.wl = 0.0               # Left Wheel
        self.linear_speed = 0.0     # Linear Speed
        self.angular_speed = 0.0    # Angular Speed
        self.l = 0.17               # Wheelbase
        self.r = 0.055              # Radius of the Wheel
        
        # Constants for error model
        self.kr = 0.15  #TODO Error coefficient for the right wheel
        self.kl = 0.13  #TODO Error coefficient for the left wheel

        # Starting pose for the puzzlebot
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0
        self.landmark_true = {"1":[2.15, 3.21], "8": [3.14, 2.6065], "3": [3.14, 2.17]}

        self.landmark = LandmarkList()
        self.landmark.landmarks = []
        self.cube_id = '2'

        # Subscribers
        self.sub_wl = self.create_subscription(Float32, '/VelocityEncL', self.cbWl, qos_profile_sensor_data)
        self.sub_wr = self.create_subscription(Float32, '/VelocityEncR', self.cbWr, qos_profile_sensor_data)
        self.aruco_sub = self.create_subscription(ArucoArray, '/aruco_info', self.aruco_callback, 10)

        # Publishers 
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        # Start the timer now
        self.start_time = self.get_clock().now()
        time_period = 0.02
        self.timer = self.create_timer(time_period, self.odom_reading)



    def cbWr(self, msg):
        self.wr = msg.data

    def cbWl(self, msg):
        self.wl = msg.data

    def aruco_callback(self, msg):
        self.aruco_info = msg
        self.landmark.landmarks = []
        if self.aruco_info.length != 0:
            for aruco in self.aruco_info.aruco_array:
                if aruco.id != self.cube_id or aruco.id == '30' or aruco.id == '31':
                    landmark = Landmark()
                    landmark.id = aruco.id
                    landmark.x, landmark.y = self.transform_cube_position(aruco.point.point)
                    self.landmark.landmarks.append(landmark)

    def transform_cube_position(self, aruco_point):
        rotation_matrix = np.array([
            [cos(self.angle), -sin(self.angle)],
            [sin(self.angle), cos(self.angle)]
        ])

        puzzlebot_coords = np.array([aruco_point.z, aruco_point.x])
        world_coords = rotation_matrix.dot(puzzlebot_coords)

        aruco_x = self.positionx + world_coords[0]
        aruco_y = self.positiony + world_coords[1]

        return aruco_x, aruco_y

    def kalman_filter(self, previous_pose):

        u_estimation = np.array([[self.positionx],[self.positiony],[self.angle]])

        # Define Jacobian matrix H_k
        H_k = np.array([
            [1, 0, -self.dt * self.linear_speed * np.sin(previous_pose["angle"])],
            [0, 1, self.dt * self.linear_speed * np.cos(previous_pose["angle"])],
            [0, 0, 1]
        ])

        Gaussian = np.array([[self.kr * abs(self.wr),   0],
                             [0,                        self.kl * abs(self.wl)]])

        Taylor = np.diag([1/2 * self.r * self.dt, 1/2 * self.r * self.dt, 1/2 * self.r * self.dt]).dot(np.array([[np.cos(previous_pose["angle"]), np.cos(previous_pose["angle"])],
                                                                                                                [np.sin(previous_pose["angle"]), np.sin(previous_pose["angle"])],
                                                                                                                [2/self.l, -2/self.l]]))
        
        Q_k = Taylor.dot(Gaussian).dot(Taylor.T)
        # Define the error matrix Q_k
        # Q_k = np.diag([self.kr * abs(self.wr) * self.dt, 
        #                self.kl * abs(self.wl) * self.dt, 
        #                (self.kr * abs(self.wr) + self.kl * abs(self.wl)) * self.dt])

        # self.angle += self.angular_speed * self.dt #+ Q_k[2][2]
        # self.positionx += self.linear_speed * np.cos(self.angle) * self.dt #+ Q_k[0][0]
        # self.positiony += self.linear_speed * np.sin(self.angle) * self.dt #+ Q_k[1][1]
        # Update covariance matrix using the previous covariance matrix
        if not hasattr(self, 'sigma'):
            self.sigma = np.eye(3)  # Initializes the covariance matrix if it hasn't been defined

        # Predict the new covariance matrix
        self.sigma_estimation = H_k.dot(self.sigma).dot(H_k.T) + Q_k

        # self.get_logger().info(f"Covariance Matrix:\n{self.sigma_estimation}")
        # self.get_logger().info("Matrix Q_k: {}".format(Q_k))

        x_diff = self.landmark_true[self.landmark.landmarks[0].id][0] - self.positionx
        y_diff = self.landmark_true[self.landmark.landmarks[0].id][1] - self.positiony

        z_estimation = np.array([[np.sqrt(x_diff**2 + y_diff**2)],
                                 [np.arctan2(y_diff, x_diff)-self.angle]])
        
        R_k = np.array([[0.01, 0.0],
                        [0.0, 0.01]])  # TODO: Change for equations from mapping

        G_k = np.array([    [-x_diff/np.sqrt(z_estimation[0][0]),     -y_diff/np.sqrt(z_estimation[0][0]),   0],
                            [y_diff/z_estimation[0][0],                -x_diff/z_estimation[0][0],             -1]])
        
        Z_linear = G_k.dot(self.sigma_estimation).dot(G_k.T) + R_k

        K_k = (self.sigma_estimation).dot(G_k.T).dot(inv(Z_linear))


        x_diff_real = self.landmark.landmarks[0].x - self.positionx
        y_diff_real = self.landmark.landmarks[0].y - self.positiony
        Z_ideal = np.array([[np.sqrt(x_diff_real**2 + y_diff_real**2)],
                            [np.arctan2(y_diff_real, x_diff_real)-self.angle]])

        u_true = u_estimation + K_k.dot(Z_ideal-z_estimation)

        self.sigma = (np.diag([1,1,1])-K_k.dot(G_k)).dot(self.sigma_estimation)
        

        # self.get_logger().info("Estimation x: {}, y: {}".format(self.positionx, self.positiony))
        # self.get_logger().info("Real x: {}, y: {}".format(u_true[0], u_true[1]))

        return u_true

    def odom_reading(self):

        #Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9
        
        self.linear_speed = self.r * (self.wr + self.wl) / 2.
        self.angular_speed = self.r * (self.wr - self.wl) / self.l

        previous_pose = {"x": self.positionx, "y": self.positiony, "angle": self.angle}

        self.angle += self.angular_speed * self.dt
        self.positionx += self.linear_speed * np.cos(self.angle) * self.dt
        self.positiony += self.linear_speed * np.sin(self.angle) * self.dt

        self.get_logger().info("Estimation x: {}, y: {}, theta: {}".format(self.positionx, self.positiony, self.angle))

        odom = Odometry()
        if not hasattr(self, 'sigma'):
            self.sigma = np.eye(3)

        if len(self.landmark.landmarks) > 0:

            u_true = self.kalman_filter(previous_pose)

            self.positionx = u_true[0][0]
            self.positiony = u_true[1][0]
            self.angle = u_true[2][0]

        sigma_full = np.zeros((6, 6))
        sigma_full[:3, :3] = self.sigma  # Fill in the 3x3 position covariance
        sigma_full[3, 3] = 0.001  # Small value for orientation around x (roll)
        sigma_full[4, 4] = 0.001  # Small value for orientation around y (pitch)
        sigma_full[5, 5] = 0.001  # Small value for orientation around z (yaw)

        # Publish odometry via odom topic
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
        # self.get_logger().info("Position y msg: {}".format(odom.pose.pose.position.y))
        self.start_time = self.get_clock().now()




def main(args=None):
    rclpy.init(args=args)
    odometry = Localisation()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()