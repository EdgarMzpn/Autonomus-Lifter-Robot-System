import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarPublisherNode(Node):
    def __init__(self):
        super().__init__('obstacule_sim')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/filtered_scan', 10)
        timer_period = 1  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Generamos un arreglo de 10 valores aleatorios para simular coordenadas LIDAR
        lidar_data = np.array([[1.5, -0.5], [1.5, 0.0], [1.5, 0.5]])  # Convertir lista de listas a arreglo numpy
        
        # Creamos el mensaje Float32MultiArray y lo llenamos con los datos del arreglo
        msg = Float32MultiArray()
        msg.data = lidar_data.flatten().tolist()  # Aplanar el arreglo numpy y convertirlo a lista
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando: %s' % str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
