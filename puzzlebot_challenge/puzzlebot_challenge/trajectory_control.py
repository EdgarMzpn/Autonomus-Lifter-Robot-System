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
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pose_pub = self.create_publisher(Pose, 'pose_ideal', 1)
        
        self.new_pose = Pose()
        self.trajectory = np.array([[0.0, 0.0]])  # Inicia en el origen
        self.distances = []  # Lista de la distancia a los puntos devueltos por el lidar
        self.distance_threshold = 0.5  # Threshold for considering obstacles
        self.linear_velocity = 0.1  # Linear velocity when avoiding obstacles
        self.angular_velocity = 0.5  # Angular velocity when avoiding obstacles
        self.distance_covered = 0.0  # Distancia cubierta por el robot
        self.last_position = None  # Última posición conocida del robot
        
        # Variables para la simulación de la pose
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0
        self.wl = 0.0
        self.wr = 0.0
        self.l = 0.19
        self.r = 0.05
        
        # Timer para actualizar la pose
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update_pose_sim)

        # Publicar pose inicial (1 metro en x)
        self.new_pose.position.x = 1.0
        self.new_pose.position.y = 0.0
        quaternion = quaternion_from_euler(0, 0, 0)
        self.new_pose.orientation.x = quaternion[0]
        self.new_pose.orientation.y = quaternion[1]
        self.new_pose.orientation.z = quaternion[2]
        self.new_pose.orientation.w = quaternion[3]
        self.pose_pub.publish(self.new_pose)

    def odometry_callback(self, msg: Odometry):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        _, _, orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        if self.last_position is not None:
            delta_distance = np.sqrt((position_x - self.last_position[0]) ** 2 + (position_y - self.last_position[1]) ** 2)
            self.distance_covered += delta_distance
        
        self.last_position = (position_x, position_y)
        self.avoid_obstacle(orientation)
        
        if not self.distances or all(d >= self.distance_threshold for d in self.distances):
            self.move_and_turn_if_necessary(orientation)
        
        self.update_trajectory(position_x, position_y, orientation)

    def lidar_callback(self, msg):
        self.distances = list(msg.data)

    def update_trajectory(self, position_x, position_y, orientation):
        self.trajectory = np.vstack([self.trajectory, [position_x, position_y]])

    def avoid_obstacle(self, orientation):
        if self.distances and min(self.distances) < self.distance_threshold:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_velocity
            self.velocity_pub.publish(twist_msg)
        else:
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = 0.0
            self.velocity_pub.publish(twist_msg)

    def move_and_turn_if_necessary(self, orientation):
        if self.distance_covered >= 1.0:
            self.distance_covered = 0.0
            new_orientation = orientation - np.pi / 2
            quaternion = quaternion_from_euler(0, 0, new_orientation)
            self.new_pose.orientation.x = quaternion[0]
            self.new_pose.orientation.y = quaternion[1]
            self.new_pose.orientation.z = quaternion[2]
            self.new_pose.orientation.w = quaternion[3]
            pose_msg = Pose()
            pose_msg.position.x = self.last_position[0]
            pose_msg.position.y = self.last_position[1]
            pose_msg.orientation = self.new_pose.orientation
            self.pose_pub.publish(pose_msg)
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -self.angular_velocity
            self.velocity_pub.publish(twist_msg)
        else:
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = 0.0
            self.velocity_pub.publish(twist_msg)

    def update_pose_sim(self):
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9
        self.angle += self.angular_speed * self.dt
        self.angle = self.angle % 6.28
        
        x_dot = self.linear_speed * np.cos(self.angle)
        y_dot = self.linear_speed * np.sin(self.angle)
        self.positionx += x_dot * self.dt
        self.positiony += y_dot * self.dt
        
        if self.linear_speed != 0. or self.angular_speed != 0:
            self.wl = ((2 * self.linear_speed) - (self.angular_speed * self.l)) / (2 * self.r)
            self.wr = ((2 * self.linear_speed) + (self.angular_speed * self.l)) / (2 * self.r)
        else:
            self.wl = 0.0
            self.wr = 0.0
        
        self.new_pose.position.x = self.positionx
        self.new_pose.position.y = self.positiony
        quaternion = quaternion_from_euler(0, 0, self.angle)
        self.new_pose.orientation.x = quaternion[0]
        self.new_pose.orientation.y = quaternion[1]
        self.new_pose.orientation.z = quaternion[2]
        self.new_pose.orientation.w = quaternion[3]
        self.pose_pub.publish(self.new_pose)
        self.start_time = self.get_clock().now()

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
