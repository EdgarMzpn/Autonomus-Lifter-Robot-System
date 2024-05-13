import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10)
        self.subscription  # Para prevenir que sea recolectado por el recolector de basura
        self.trajectory = np.array([[0, 0]])  # Inicia en el origen
        self.landmarks = []  # Lista de landmarks (puntos de referencia) en el mapa

    def odometry_callback(self, msg):
        # Obtener la posición y la orientación del mensaje de odometría
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        
        # Obtener el ángulo actual del quaternion
        quaternion = msg.pose.pose.orientation
        _, _, orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        # Actualizar la trayectoria del robot
        self.update_trajectory(position_x, position_y, orientation)
        
        # Actualizar el mapa (en este caso, solo se agregan puntos de referencia en ciertos pasos)
        if len(self.trajectory) % 5 == 0:
            self.landmarks.append(self.trajectory[-1])

    def update_trajectory(self, position_x, position_y, orientation):
        # Actualiza la trayectoria del robot usando la posición y la orientación
        x, y = self.trajectory[-1]
        distance = np.sqrt((position_x - x)**2 + (position_y - y)**2)
        new_x = x + distance * np.cos(orientation)
        new_y = y + distance * np.sin(orientation)
        self.trajectory = np.vstack([self.trajectory, [new_x, new_y]])

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

    slam_node = SLAMNode()

    rclpy.spin(slam_node)

    slam_node.plot_map()

    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
