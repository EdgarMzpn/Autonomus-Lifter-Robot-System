import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from puzzlebot_msgs.msg import Arucoinfo, ArucoArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import enum
from bug2 import Bug2Controller

class StateMachine(enum.Enum):
    FIND_CORNER = 1
    WANDER = 2
    GO_TO_TARGET = 3
    HANDLE_OBJECT = 4
    STOP = 5


class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('Trajectory_Control')
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/filtered_scan', self.lidar_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoArray, '/aruco_info', self.aruco_callback, 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.current_pose.position.x = 1.0
        self.current_pose.position.y = 0.0
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = 0.0
        self.current_pose.orientation.w = 1.0
        self.current_angle = 0.0

        self.current_state = StateMachine.FIND_CORNER
        
        self.discharge_area = PoseStamped()
        self.discharge_area.header.frame_id = "world"
        self.discharge_area.position.x = 0.0
        self.discharge_area.position.y = 0.0

        self.goal = PoseStamped()
        self.goal.header.frame_id = "world"
        self.goal.pose.position.x = 2.0
        self.goal.pose.position.y = 2.0

        self.cmd_vel = 0.0

        self.there_is_aruco = False
        self.aruco_info = ArucoArray()

        self.target_area_id = None
        self.distances = []  # Lista de la distancia a los puntos devueltos por el lidar
        self.distance_covered = 0.0  # Distancia cubierta por el robot

        # self.corner_finder = Corner_Finder()
        # self.wander = Wander()
        # self.handle_object = Handle()
        self.target = Bug2Controller()
        self.target.goal_callback(self.goal)
        
        # Timer para actualizar la pose
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.run)
    def odometry_callback(self, msg):
        self.current_pose.pose = msg
        self.target.odom_callback(msg)

    def aruco_callback(self, msg):
        self.aruco_info = msg
        self.there_is_aruco = True
    
    def lidar_callback(self, msg):
        # Update the distances to obstacles based on laser scan data
        self.distances = np.array(msg.ranges)
        self.target.scan_callback(msg)


    def run(self):
        
        # if self.current_state is StateMachine.FIND_CORNER:
        #     if self.corner_finder.finish:
        #         self.current_state = StateMachine.WANDER
        #     else: self.corner_finder.run()

        # elif self.current_state is StateMachine.WANDER:
        #     if self.aruco_info.id:
        #         self.current_state = StateMachine.GO_TO_TARGET
        #     else: self.wander.run()

        if self.current_state is StateMachine.GO_TO_TARGET:
            if self.aruco_info.point.z < 0.10:
                if self.aruco_info.aruco_array[0].id == self.target_area_id:
                    self.discharge_area.position.x = self.current_position.position.x + self.aruco_info.point.z
                    self.discharge_area.position.y = self.current_position.position.y + self.aruco_info.point.x
                    self.current_state = StateMachine.WANDER
                else:
                    self.current_state = StateMachine.HANDLE_OBJECT
            else: self.target.run()

        # elif self.current_state is StateMachine.HANDLE_OBJECT:
        #     if self.handle.picked:
        #         self.current_state = StateMachine.GO_TO_TARGET
        #     elif self.handle.placed: 
        #         self.current_state = StateMachine.STOP
        #     else:
        #         self.handle.run()
        elif self.current_state is StateMachine.STOP:
            self.target.run()
        
        self.there_is_aruco = False
        


            




def main(args=None):
    rclpy.init(args=args)
    slam_node = TrajectoryControl()
    rclpy.spin(slam_node)
    slam_node.plot_map()
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
