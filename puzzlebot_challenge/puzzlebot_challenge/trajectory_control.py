import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion

class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('trajectory_control')
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/goal', self.goal_callback, 10)
        self.ideal_pose_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.new_cmd_vel = Twist()
        self.trajectory = np.array([[0.0, 0.0]])  # Inicia en el origen
        self.distances = []  # Lista de la distancia a los puntos devueltos por el lidar
        self.angles = []  # Lista de los ángulos correspondientes
        self.distance_threshold = 0.5  # Umbral para considerar obstáculos
        self.distance_covered = 0.0  # Distancia cubierta por el robot
        self.last_position = None  # Última posición conocida del robot
        self.goal = None  # Añadido para inicializar goal
        self.current_pose = None  # Añadido para inicializar current_pose

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

    def lidar_callback(self, msg: LaserScan):
        self.distances = np.array(msg.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(self.distances))

    def goal_callback(self, msg: Pose):
        self.goal = msg

    def update_trajectory(self, position_x, position_y, orientation):
        self.trajectory = np.vstack([self.trajectory, [position_x, position_y]])

    def avoid_obstacle(self, position_x, position_y, orientation):
        if self.goal is None:
            return

        if not all(np.isfinite([self.goal.position.x, self.goal.position.y, position_x, position_y])):
            return

        attractive_force = 0.1 * np.array([self.goal.position.x - position_x, self.goal.position.y - position_y])
        repulsive_force = np.array([0.0, 0.0])

        for distance, angle in zip(self.distances, self.angles):
            if 0 < distance < self.distance_threshold:
                global_angle = orientation + angle
                repulsive_force += 0.5 * np.array([np.cos(global_angle), np.sin(global_angle)]) / distance**2

        if not all(np.isfinite(repulsive_force)):
            return

        total_force = attractive_force + repulsive_force
        norm = np.linalg.norm(total_force)
        if norm != 0:
            total_force /= norm

        self.new_cmd_vel.linear.x = total_force[0]
        self.new_cmd_vel.angular.z = total_force[1]

        self.ideal_pose_pub.publish(self.new_cmd_vel)

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
