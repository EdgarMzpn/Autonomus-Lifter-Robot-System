import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarPublisherNode(Node):
    def __init__(self):
        super().__init__('obstacle_sim')
        # Crear el publicador
        self.publisher_ = self.create_publisher(Float32MultiArray, '/filtered_scan', 10)
        # Establecer el periodo del temporizador en segundos
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            # Datos LIDAR simulados
            lidar_data = np.array([[11.5, 7.5], [11.0, 6.0], [2.5, 8.5]])
            
            # Crear el mensaje Float32MultiArray y llenarlo con los datos
            msg = Float32MultiArray()
            msg.data = lidar_data.flatten().tolist()  # Aplanar el arreglo numpy y convertirlo a lista
            
            # Publicar el mensaje
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando: %s' % str(msg.data))
        except Exception as e:
            self.get_logger().error('Error al publicar datos LIDAR: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
