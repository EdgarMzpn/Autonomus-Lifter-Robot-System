import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np

class LidarPublisherNode(Node):
    def __init__(self):
        super().__init__('obstacle_sim')
        # Create publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, '/filtered_scan', 10)
        # Create subscription
        self.arrive_sub = self.create_subscription(Bool, '/arrive', self.arrive_callback, 10)
        # Initialize 'arrive' state
        self.arrive = False
        self.second_publish_done = False
        self.third_publish_done = False
        # Set timer period in seconds
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def arrive_callback(self, msg):
        self.arrive = msg.data
        #self.get_logger().info(f'Received arrive: {self.arrive}')
        if self.arrive:
            if not self.second_publish_done:
                self.send_second_coordinates()
            elif not self.third_publish_done:
                self.send_third_coordinates()

    def send_second_coordinates(self):
        # New simulated LIDAR data (second list)
        new_lidar_data = np.array([[0.6, 0.5], [1.0, 1.0], [0.5, 0.5]])
        
        # Create Float32MultiArray message and fill it with new data
        msg = Float32MultiArray()
        msg.data = new_lidar_data.flatten().tolist()  # Flatten numpy array and convert to list
        
        # Publish the message
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing second new data: %s' % str(msg.data))
        self.second_publish_done = True
        self.timer.cancel()

    def send_third_coordinates(self):
        # New simulated LIDAR data (third list)
        third_lidar_data = np.array([[1.0, 1.0], [0.6, 0.6], [0.5, 0.5]])
        
        # Create Float32MultiArray message and fill it with third data
        msg = Float32MultiArray()
        msg.data = third_lidar_data.flatten().tolist()  # Flatten numpy array and convert to list
        
        # Publish the message
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing third new data: %s' % str(msg.data))
        self.third_publish_done = True
        self.timer.cancel()

    def timer_callback(self):
        try:
            if not self.arrive:
                # Simulated LIDAR data (first list)
                lidar_data = np.array([[1.5, 1.5], [1.0, 1.0], [0.5, 0.5]])
                
                # Create Float32MultiArray message and fill it with data
                msg = Float32MultiArray()
                msg.data = lidar_data.flatten().tolist()  # Flatten numpy array and convert to list
                
                # Publish the message
                self.publisher_.publish(msg)
                #self.get_logger().info('Publishing: %s' % str(msg.data))
        except Exception as e:
            self.get_logger().error('Error publishing LIDAR data: %s' % str(e))

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

