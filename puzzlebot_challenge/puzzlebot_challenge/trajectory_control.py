# Ros libraries
import rclpy
from rclpy.node import Node

# Ros messages
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

#Custom messages
from puzzlebot_msgs.msg import Arucoinfo

# Math libraries
import numpy as np
from tf_transformations import euler_from_quaternion

class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('Trajectory_Control')

        # Initialize subscribers
        self.aruco_info_sub = self.create_publisher(Arucoinfo, '/aruco_info', self.aruco_info_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/goal', self.goal_callback, 10)
        self.lidar_sub = self.create_subscription(Float32MultiArray, '/filtered_scan', self.lidar_callback, 10)
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.pose_sub = self.create_subscription(Pose, '/pose', self.pose_callback, 10)

        # Initialize publishers
        self.pose_pub = self.create_publisher(Pose, '/pose_ideal', 1)
        
        # Initialize publish message
        self.new_pose = Pose()

        # Initialize variables
        self.distances = []  # Lidar points array
        self.distance_threshold = 0.5  # Threshold for obstacle detection
        self.distance_covered = 0.0  # Robot's covered distance
        
        # Timer for pose update
        self.start_time = self.get_clock().now()

    # Callback functions
    def aruco_info_callback(self, msg: Arucoinfo):
        self.aruco_info = msg.data

    def goal_callback(self, msg: Pose):
        self.goal = msg

    def lidar_callback(self, msg):
        self.distances = list(msg.data)

    def odometry_callback(self, msg: Odometry):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        _, _, orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        if self.last_position is not None:
            delta_distance = np.sqrt((position_x - self.last_position[0]) ** 2 + (position_y - self.last_position[1]) ** 2)
            self.distance_covered += delta_distance

        self.last_position = (position_x, position_y)
        self.avoid_obstacle(position_x, position_y, orientation)

        self.update_trajectory(position_x, position_y, orientation)

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    # States functions
    def wander(self):
        forward_offset = 0.1
        self.new_pose.position.x = (self.current_pose.position.x + forward_offset) * np.cos(self.current_pose.orientation.z)
        self.new_pose.position.y = (self.current_pose.position.y + forward_offset) * np.sin(self.current_pose.orientation.z)
        
    
    def align_to_goal(self):
        return 0
    
    def align_to_aruco(self):
        return 0

    def pick_up_aruco(self):
        return 0
    
    def got_to_goal(self):
        return 0

    def place_aruco(self):
        return 0
    
    # Utilities
    def found_obstacle(self):
        for obstacle in self.distances:
            x_distance = self.disatnces[obstacle][0] * np.cos(self.current_pose.orientation.z)
            y_distance = self.disatnces[obstacle][1] * np.sin(self.current_pose.orientation.z)
            distance = np.sqrt(x_distance**2 + y_distance**2)



def main(args=None):
    rclpy.init(args=args)
    slam_node = TrajectoryControl()
    rclpy.spin(slam_node)
    slam_node.plot_map()
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
