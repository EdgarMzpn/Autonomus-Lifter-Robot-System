import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node, Float32
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32

class TrayectoryControl(Node):
    def __init__(self):
        super().__init__('Trayectory_Control')
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.lidar_sub = self.create_subscription(Float32, '/filtered_scan', self.lidar_callback, 10)
        self.velocity_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.pose_pub = self.create_publisher(Pose, 'pose_ideal', 1)
        self.new_pose = Pose()

        self.trajectory = np.array([[0, 0]])  # Inicia en el origen
        self.landmarks = []  # Lista de landmarks (puntos de referencia) en el mapa
        self.distances = []  # Lista de la distancia a los puntos devueltos por el lidar
        
        self.distance_threshold = 0.5  # Threshold for considering obstacles
        self.angular_velocity_threshold = 0.05  # Threshold for angular velocity
        self.linear_velocity = 0.1  # Linear velocity when avoiding obstacles

    def odometry_callback(self, msg):
        # Obtener la posición y la orientación del mensaje de odometría
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        
        # Obtener el ángulo actual del quaternion
        quaternion = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        # Actualizar la trayectoria del robot
        self.update_trajectory(position_x, position_y, self.orientation)
        
        # Actualizar el mapa (en este caso, solo se agregan puntos de referencia en ciertos pasos)
        if len(self.trajectory) == 20:
            self.landmarks.append(self.trajectory[-1])
            self.trajectory = []

    def lidar_callback(self, msg):
        self.distances = msg

    def cmd_vel_callback(self, msg):
        self.angular_velocity = msg.angular.z

    def update_trajectory(self, position_x, position_y, orientation):
        # Actualiza la trayectoria del robot usando la posición y la orientación

        if self.angular_velocity < self.angular_velocity_threshold:
            x, y = self.trajectory[-1]
            distance = np.sqrt((position_x - x)**2 + (position_y - y)**2)
            new_x = int((x + distance * np.cos(orientation)) * 100) # cm
            new_y = int((y + distance * np.sin(orientation)) * 100) # cm
            self.trajectory = np.vstack([self.trajectory, [new_x, new_y]])
    
    def avoid_obstacle(self):
        if (self.distances[0] < self.distance_threshold and self.distances[-1] < self.distance_threshold) or self.trajectory[-1] in self.landmarks:
            # If obstacle detected or approaching a landmark, adjust orientation to avoid it
            self.new_pose.orientation.z = self.orientation + np.pi / 2
            # Reduce linear velocity to slow down while avoiding obstacle
            self.new_pose.linear.x = self.linear_velocity
            self.pose_pub.publish(self.new_pose)

    def plot_map(self):
        plt.figure(figsize=(8, 6))
        plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], 'b-', label='Trayectoria del robot')
        for landmark in self.landmarks:
            plt.plot(landmark[0], landmark[1], 'ro', markersize=8)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Mapa y trayectoria del robot')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    slam_node = TrayectoryControl()

    rclpy.spin(slam_node)

    slam_node.plot_map()

    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
