import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import numpy as np

class GoalSimulation(Node):
    def __init__(self):
        super().__init__('goal_simulation')
        self.pose_pub = self.create_publisher(Pose, 'goal', 1)
        self.lidar_sub = self.create_subscription(Float32MultiArray, '/filtered_scan', self.lidar_callback, 10)
        self.furthest_point = None

        # Crear un temporizador para publicar la pose periódicamente
        timer_period = 1.0  # segundos
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def lidar_callback(self, msg):
        # Convertir la lista recibida a un arreglo numpy y reshape a un arreglo de Nx2
        lidar_data = np.array(msg.data).reshape(-1, 2)

        # Calcular la distancia de cada punto al origen (0,0)
        distances = np.linalg.norm(lidar_data, axis=1)

        # Encontrar el índice del punto más alejado
        furthest_index = np.argmax(distances)

        # Guardar el punto más alejado
        self.furthest_point = lidar_data[furthest_index]
        self.get_logger().info(f'Punto más alejado: {self.furthest_point}')

    def publish_pose(self):
        if self.furthest_point is not None:
            # Crear el mensaje Pose con la coordenada más alejada
            pose = Pose()
            pose.position.x = float(self.furthest_point[0])
            pose.position.y = float(self.furthest_point[1])
            pose.position.z = 0.0  # Asumimos que z es 0 en un entorno 2D
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            # Publicar el mensaje Pose
            self.pose_pub.publish(pose)
            self.get_logger().info(f'Publicando goal: x={pose.position.x}, y={pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    goal_sim_node = GoalSimulation()
    rclpy.spin(goal_sim_node)
    goal_sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
