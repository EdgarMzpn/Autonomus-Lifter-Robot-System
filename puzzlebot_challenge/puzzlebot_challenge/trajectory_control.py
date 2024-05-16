import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('Trajectory_Control')
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.lidar_sub = self.create_subscription(Float32MultiArray, '/filtered_scan', self.lidar_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/goal', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(Pose, '/pose', self.pose_callback, 10)
        self.pose_pub = self.create_publisher(Pose, '/pose_ideal', 1)
        
        self.new_pose = Pose()
        self.trajectory = np.array([[0.0, 0.0]])  # Inicia en el origen
        self.distances = []  # Lista de la distancia a los puntos devueltos por el lidar
        self.distance_threshold = 0.5  # Threshold for considering obstacles
        self.distance_covered = 0.0  # Distancia cubierta por el robot
        self.last_position = None  # Última posición conocida del robot
        
        # Timer para actualizar la pose
        self.start_time = self.get_clock().now()

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

    def lidar_callback(self, msg):
        self.distances = list(msg.data)

    def goal_callback(self, msg: Pose):
        self.goal = msg

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def update_trajectory(self, position_x, position_y, orientation):
        self.trajectory = np.vstack([self.trajectory, [position_x, position_y]])

    def avoid_obstacle(self, position_x, position_y, orientation):
        # Check for valid inputs
        if not all(np.isfinite([self.goal.position.x, self.goal.position.y, position_x, position_y])):
            return
        
        attractive_force = 0.1 * np.array([self.goal.position.x - position_x, self.goal.position.y - position_y])
        repulsive_force = np.array([0.0, 0.0])

        for distance in self.distances:
            # Check for division by zero and valid distance
            if distance > 0 and distance < self.distance_threshold:
                angle = orientation + np.pi / 2  # Convert orientation to obstacle-facing angle
                repulsive_force += 0.5 * np.array([np.cos(angle), np.sin(angle)]) / distance**2
        
        # Check for NaN or infinite values
        if not all(np.isfinite(repulsive_force)):
            return
        
        total_force = attractive_force + repulsive_force
        total_force /= np.linalg.norm(total_force)  # Normalize force vector

        # Update robot's pose based on the calculated force
        self.new_pose.position.x = position_x + total_force[0]
        self.new_pose.position.y = position_y + total_force[1]
        self.new_pose.position.z = 0.0  # Assuming 2D navigation, z = 0

        # Publish the new pose
        self.pose_pub.publish(self.new_pose)

    def plot_map(self):
        plt.figure(figsize=(8, 6))
        plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], 'b-', label='Trayectoria del robot')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Mapa y trayectoria del robot')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    slam_node = TrajectoryControl()
    rclpy.spin(slam_node)
    slam_node.plot_map()
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
