import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from puzzlebot_msgs.msg import Arucoinfo, ArucoArray, LandmarkList, Landmark
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import enum
from math import sin, cos, inf, pi
from numpy import linspace

class StateMachine(enum.Enum):
    FIND_LANDMARK = 1
    WANDER = 2
    GO_TO_TARGET = 3
    HANDLE_OBJECT = 4
    STOP = 5

class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('Trajectory_Control')

        # Suscripciones
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/filtered_scan', self.lidar_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoArray, '/aruco_info', self.aruco_callback, 10)
        self.arrived_sub = self.create_subscription(Bool, '/arrived', self.arrived_callback, 10)

        # Publicadores
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 1)
        self.landmarks_pub = self.create_publisher(LandmarkList, '/landmarks', 1)
        self.bug_pub = self.create_publisher(Bool, '/bug2_run', 1)
        
        # Inicialización de la pose y ángulo actual
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0
        self.current_pose.pose.orientation.w = 1.0
        self.current_angle = 0.0

        # Estado inicial de la máquina de estados
        self.current_state = StateMachine.FIND_LANDMARK
        
        # Área de descarga y objetivo
        self.discharge_area = PoseStamped()
        self.discharge_area.header.frame_id = "world"
        self.discharge_area.pose.position.x = 0.0
        self.discharge_area.pose.position.y = 0.0

        self.goal = PoseStamped()
        self.goal.header.frame_id = "world"
        self.goal.pose.position.x = 2.0
        self.goal.pose.position.y = 2.0

        # Otras variables
        self.cmd_vel = None
    
        self.cube_id = '2'

        self.aruco_info = ArucoArray()
        self.aruco_info.length = 0
        self.aruco_info.aruco_array = []

        self.target_area_id = None
        self.distances = []
        self.distance_covered = 0.0

        self.extent = 1.0  # Define la variable extent

        # Timer para actualizar la pose
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.run)
        self.arrived = None
        self.wandergoal = False

    def odometry_callback(self, msg: Odometry):
        self.current_pose.pose = msg.pose.pose
        orientation_euler = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.current_angle = orientation_euler[2]

    def aruco_callback(self, msg):
        self.aruco_info = msg
    def lidar_callback(self, msg):
        self.distances = np.array(msg.ranges)

    def transform_cube_position(self, aruco_point):
        rotation_matrix = np.array([
            [cos(self.current_angle), -sin(self.current_angle)],
            [sin(self.current_angle), cos(self.current_angle)]
        ])

        puzzlebot_coords = np.array([aruco_point.z, aruco_point.x])
        world_coords = rotation_matrix.dot(puzzlebot_coords)

        aruco_x = self.current_pose.pose.position.x + world_coords[0]
        aruco_y = self.current_pose.pose.position.y + world_coords[1]

        return aruco_x, aruco_y

    def arrived_callback(self, msg):
        self.arrived = msg.data

    def run(self):
        if self.current_state is StateMachine.FIND_LANDMARK:
            self.get_logger().info(f"Entering FIND_LANDMARK")
            if self.aruco_info:
                stop_spin_msg = Twist()
                self.velocity_pub.publish(stop_spin_msg)
                self.current_state = StateMachine.WANDER
            else:
                spin_msg = Twist()
                spin_msg.angular.z = 0.05  # Velocidad angular en radianes por segundo
                self.velocity_pub.publish(spin_msg)

        elif self.current_state is StateMachine.WANDER:
            self.get_logger().info(f"Entering Wander")
            if self.aruco_info.length > 0:
                if self.aruco_info.aruco_array[0].id == self.cube_id:
                    self.goal.pose.position.x, self.goal.pose.position.y = self.transform_cube_position(self.aruco_info.aruco_array[0].point.point)
                    self.goal_pub.publish(self.goal)
                    self.current_state = StateMachine.GO_TO_TARGET

            #elif np.any(self.distances) and self.wandergoal == False:
                #angles = np.linspace(-pi/2, pi/2, len(self.distances))
                #points = [r * sin(theta) if (theta < -1.0 or theta > 1.0) else inf for r, theta in zip(self.distances, angles)]
                #new_ranges = [r if abs(y) < self.extent else inf for r, y in zip(self.distances, points)]
                #self.distances = new_ranges

                # Filtrar los valores válidos
                #valid_distances = [r for r in self.distances if not np.isinf(r) and not np.isnan(r)]

                #if valid_distances:
                    #max_index = np.argmax(valid_distances)
                    #max_range = valid_distances[max_index]

                    #self.get_logger().info(f"Wandering: Max distance = {max_range:.2f} meters at angle = {np.degrees(angles[max_index]):.2f} degrees")

                    #robot_x = self.current_pose.pose.position.x
                    #robot_y = self.current_pose.pose.position.y
                    #robot_theta = self.current_angle

                    #goal_x = robot_x + max_range * cos(robot_theta + angles[max_index])
                    #goal_y = robot_y + max_range * sin(robot_theta + angles[max_index])

                    #self.goal.pose.position.x = goal_x
                    #self.goal.pose.position.y = goal_y
                    #self.goal_pub.publish(self.goal)
                    #self.get_logger().info(f"Published goal: x = {goal_x:.2f}, y = {goal_y:.2f}")
                    #self.wandergoal = True
                    #self.current_state = StateMachine.GO_TO_TARGET
                #else:
                    #self.get_logger().info("Wandering: No valid max range found")

        elif self.current_state is StateMachine.GO_TO_TARGET:
            self.get_logger().info(f"Entering GO_TO_TARGET")
            # if self.aruco_info.length == 0 and self.arrived == True:
            #     spin_msg = Twist()
            #     spin_msg.angular.z = 0.05  # Velocidad angular en radianes por segundo
            #     self.velocity_pub.publish(spin_msg)
            #     self.current_state = StateMachine.FIND_LANDMARK
            if self.aruco_info:
                if self.aruco_info.aruco_array[0].point.point.z < 0.35:
                    self.current_state = StateMachine.HANDLE_OBJECT
            else:
                self.bug_pub.publish(Bool(data=True))

        elif self.current_state is StateMachine.HANDLE_OBJECT:
            self.get_logger().info(f"Entering HANDLE_OBJECT")
            # Funciones comentadas
            # if self.handle.picked:
            #     self.current_state = StateMachine.GO_TO_TARGET
            # elif self.handle.placed:
            #     self.current_state = StateMachine.STOP
            # else:
            #     self.handle.run()
            pass

        elif self.current_state is StateMachine.STOP:
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.velocity_pub.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    slam_node = TrajectoryControl()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
