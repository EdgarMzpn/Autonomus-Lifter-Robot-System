import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np

class GoalSimulation(Node):
    def __init__(self):
        super().__init__('goal_simulation')
        self.pose_pub = self.create_publisher(Pose, 'goal', 1)
        self.lidar_sub = self.create_subscription(Float32MultiArray, 'filtered_scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.furthest_point = None
        self.current_position = np.array([0.0, 0.0])
        self.last_lidar_data = None
        self.current_goal = Pose()

        # Initialize an initial goal
        self.current_goal.position.x = 0.0
        self.current_goal.position.y = 0.0
        self.current_goal.position.z = 0.0
        self.current_goal.orientation.x = 0.0
        self.current_goal.orientation.y = 0.0
        self.current_goal.orientation.z = 0.0
        self.current_goal.orientation.w = 1.0
        self.pose_pub.publish(self.current_goal)
        self.get_logger().info(f'Published initial goal: x={self.current_goal.position.x}, y={self.current_goal.position.y}, z={self.current_goal.position.z}')

    def odom_callback(self, msg):
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.get_logger().info(f'Current position updated: x={self.current_position[0]}, y={self.current_position[1]}')

    def lidar_callback(self, msg):
        # Convert the received list to a numpy array and reshape to an Nx2 array
        lidar_data = np.array(msg.data).reshape(-1, 2)

        # Check if the new lidar data is different from the last lidar data
        if not np.array_equal(lidar_data, self.last_lidar_data):
            self.last_lidar_data = lidar_data
            self.get_logger().info(f'New lidar data received: {lidar_data}')

            # Calculate the distance of each point to the origin (0,0)
            distances = np.linalg.norm(lidar_data, axis=1)

            # Find the index of the furthest point
            furthest_index = np.argmax(distances)

            # Update the furthest point
            self.furthest_point = lidar_data[furthest_index]

            # Publish the new goal based on the furthest point
            self.update_and_publish_goal()

    def update_and_publish_goal(self):
        if self.furthest_point is not None:
            # Create a Pose message with the furthest point as the goal
            self.current_goal.position.x = self.current_position[0] + float(self.furthest_point[0])
            self.current_goal.position.y = self.current_position[1] + float(self.furthest_point[1])
            self.current_goal.position.z = 0.0  # Assume z is 0 in a 2D environment
            self.current_goal.orientation.x = 0.0
            self.current_goal.orientation.y = 0.0
            self.current_goal.orientation.z = 0.0
            self.current_goal.orientation.w = 1.0

            # Publish the Pose message
            self.pose_pub.publish(self.current_goal)
            self.get_logger().info(f'Published new goal: x={self.current_goal.position.x}, y={self.current_goal.position.y}')

def main(args=None):
    rclpy.init(args=args)
    goal_sim_node = GoalSimulation()
    rclpy.spin(goal_sim_node)
    goal_sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
