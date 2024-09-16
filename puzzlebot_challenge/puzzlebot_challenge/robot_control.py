import rclpy
import numpy as np
import enum
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from puzzlebot_msgs.msg import Arucoinfo, ArucoArray, LandmarkList
from tf_transformations import euler_from_quaternion
from math import sin, cos, inf, pi

class StateMachine(enum.Enum):
    """
    Enum representing the states of the state machine.
    
    States:
        FIND_LANDMARK: Rotate the robot until it finds a marker.
        GO_TO_TARGET: Move the robot towards target.
        HANDLE_OBJECT: Operate the lifter peripheral to interact with the objects
        STOP: Stop the robot when the mission is fulfilled.
    """
    FIND_LANDMARK = 1
    GO_TO_TARGET = 2
    HANDLE_OBJECT = 3
    STOP = 4

class RobotControl(Node):
    """
    A ROS2 node that controls the trajectory of a robot based on sensor inputs and state machine logic.

    This class subscribes to various topics for odometry, lidar, and ArUco marker information,
    and publishes velocity commands, goals, and other relevant information.
    """
    def __init__(self):
        super().__init__('Robot Control')

        # Initialize subscribers for odometry, markers info and other nodes feedback
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoArray, '/aruco_info', self.aruco_callback, 10)
        self.arrived_sub = self.create_subscription(Bool, '/arrived', self.arrived_callback, 10)
        self.handled_sub = self.create_subscription(Bool, '/handled_aruco', self.handled_callback, 10)
        self.object_sub = self.create_subscription(String, '/object_state', self.object_state_callback, 10)

        # Initialize publisher for velocity commands, goal, and node activation messages
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 1)
        self.bug_pub = self.create_publisher(Bool, '/bug2_run', 1)
        self.handle_run_pub = self.create_publisher(Bool, '/handle_run', 1)
        self.handle_pub = self.create_publisher(Int32, '/handle', 1)
                
        # Initialize of current pose
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0
        self.current_pose.pose.orientation.w = 1.0
        self.current_angle = 0.0

        # Initialize goal pose
        self.goal = PoseStamped()
        self.goal.header.frame_id = "world"
        self.goal.pose.position.x = 0.0
        self.goal.pose.position.y = 0.0

        # Initialize parameters
        self.current_state = StateMachine.FIND_LANDMARK
        self.cmd_vel = None
        self.cube_id = '2'
        self.object_state = ""
        self.arrived = None
        self.carga = False

        # Initialize aruco info 
        self.aruco_info = ArucoArray()
        self.aruco_info.length = 0
        self.aruco_info.aruco_array = []
        self.goal_ids = {
            'A': '1',
            'B': '8',
            'C': '3'

        }
        self.desired_goal = 'A'

        # Initialize stations general position
        self.convergence_point = PoseStamped()
        self.convergence_point.pose.position.x = 2.2
        self.convergence_point.pose.position.y = 2.1
        self.convergence_point.pose.position.z = 0.0
        self.convergence_point.header.frame_id = 'world'

        # Initialize timer 
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.run)

      
    def odometry_callback(self, msg):
        """
        Callback function for the odometry topic.

        Updates the current pose and orientation of the robot based on the odometry message.

        Args:
            msg (Odometry): The odometry message containing the robot's current pose and orientation.
        """
        self.current_pose.pose = msg.pose.pose
        orientation_euler = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.current_angle = orientation_euler[2]

    def aruco_callback(self, msg):
        """
        Callback function for aruco markers info 
        
        Stores information of relevant markers 

        Args:
            msg (ArucoArray): Array containing individual information of each aruco such as id and position
        """
        self.aruco_info = msg
        for aruco in self.aruco_info.aruco_array:
            if aruco.id == self.goal_ids[self.desired_goal] or aruco.id == self.cube_id:
                self.aruco_info.aruco_array[0] = aruco 

    def handled_callback(self, msg):
        """
        Callback function for object interaction with the robot

        Args:
            msg (Bool): Wheter the robot has finished interacting with the object yet
        """
        self.carga = msg.data

    def object_state_callback(self, msg):
        """
        Callback function for last interaction with the object

        Args:
            msg (String): Last interaction with object
        """
        self.object_state = msg.data

    def arrived_callback(self, msg):
        """
        Callback function for navigation node task completion 

        Args:
            msg (Bool): Wheter the navigation algorithm has reached its goal or not
        """
        self.arrived = msg.data

    def transform_cube_position(self, aruco_point):
        """
        Transform cube position from camera puzzlebot local frame to global frame using rotation matrix

        Args:
            aruco_point (Point): Cube position coordinates measured by camera

        Returns:
            aruco_x, aruco_y: Tuple of relevant coordinates in world frame
        """
        rotation_matrix = np.array([
            [cos(self.current_angle), -sin(self.current_angle)],
            [sin(self.current_angle), cos(self.current_angle)]
        ])

        puzzlebot_coords = np.array([aruco_point.z, aruco_point.x])
        world_coords = rotation_matrix.dot(puzzlebot_coords)

        aruco_x = self.current_pose.pose.position.x + world_coords[0]
        aruco_y = self.current_pose.pose.position.y + world_coords[1]

        return aruco_x, aruco_y

    def run(self):
        """
        Main loop to control robot behavior and decisions 
        """

        if self.current_state is StateMachine.FIND_LANDMARK:
            if self.aruco_info.length > 0:
                if self.aruco_info.aruco_array[0].id == self.cube_id:
                    stop_spin_msg = Twist()
                    self.goal.pose.position.x, self.goal.pose.position.y = self.transform_cube_position(self.aruco_info.aruco_array[0].point.point)
                    self.velocity_pub.publish(stop_spin_msg)
                    self.goal_pub.publish(self.goal)
                    self.current_state = StateMachine.GO_TO_TARGET
                elif self.object_state == 'lifted' and self.arrived and self.aruco_info.aruco_array[0].id == self.goal_ids[self.desired_goal]:
                    stop_spin_msg = Twist()
                    self.velocity_pub.publish(stop_spin_msg)
                    self.handle_pub.publish(Int32(data = 1))
                    self.object_state = 'dropped'
                    self.current_state = StateMachine.HANDLE_OBJECT
            else:
                spin_msg = Twist()
                spin_msg.angular.z = -0.05 
                spin_msg.linear.x = 0.0
                self.velocity_pub.publish(spin_msg)
            
        elif self.current_state is StateMachine.GO_TO_TARGET:
            if self.aruco_info.length > 0 and self.carga == False:
                if self.aruco_info.aruco_array[0].id == self.cube_id and self.aruco_info.aruco_array[0].point.point.z < 0.5:
                    stop_spin_msg = Twist()
                    self.velocity_pub.publish(stop_spin_msg)
                    self.handle_pub.publish(Int32(data=0))
                    self.object_state = 'lifted'
                    self.current_state = StateMachine.HANDLE_OBJECT
                else:
                    self.bug_pub.publish(Bool(data=True))
            elif self.object_state == 'lifted' and self.arrived:
                self.current_state = StateMachine.FIND_LANDMARK
            else:
                self.bug_pub.publish(Bool(data=True))

        elif self.current_state is StateMachine.HANDLE_OBJECT:
            if self.carga:
                if self.object_state == "lifted":
                    self.goal = self.convergence_point
                    self.goal_pub.publish(self.goal)
                    self.current_state = StateMachine.GO_TO_TARGET
                    self.carga = False
                elif self.object_state == "dropped":
                    self.goal.pose.position.x = 0.0
                    self.goal.pose.position.y = 0.0
                    self.goal_pub.publish(self.goal)
                    self.current_state = StateMachine.STOP
                else: 
                    self.handle_run_pub.publish(Bool(data=True))
            else:
                self.get_logger().info(f"Handle running trough pub")
                self.handle_run_pub.publish(Bool(data = True))
                

        elif self.current_state is StateMachine.STOP:
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.velocity_pub.publish(stop)

    def destroy_node(self):
        """
        Function for shutting down the wheels motors upon interruptions
        """
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.velocity_pub.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
