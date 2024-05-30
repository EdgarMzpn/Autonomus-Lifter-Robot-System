import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from puzzlebot_msgs.msg import LandmarkList, Landmark

class LandmarkDetector(Node):

    def __init__(self):
        super().__init__('landmark_detector')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.landmark_sub = self.create_subscription(LandmarkList, '/landmark', self.landmarks_callback, 10)

        self.processed_landmark_pub = self.create_publisher(Landmark, '/processed_landmark', 10)

        # Initialize tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_pose = PoseStamped()
        self.current_angle = 0.0
        self.distances = []
        self.landmarks = {}

    def lidar_callback(self, msg):
        self.distances = np.array(msg.ranges)
        self.get_logger().info(f"The distances are: {self.distances}")

    def landmarks_callback(self, msg):
        # Process the received landmark data
        for landmark in msg.landmarks:
            distance, orientation = self.calculate_distance_orientation(landmark)
            # Transform landmark to map frame
            transformed_landmark_pose = self.transform_landmark_to_map(landmark)
            if transformed_landmark_pose:
                self.landmarks[landmark.id] = (transformed_landmark_pose.pose.position.x, transformed_landmark_pose.pose.position.y)
            self.publish_processed_landmark(landmark.id, distance, orientation)
        
    def odom_callback(self, msg):
        # Update the current pose and angle based on odometry data
        self.current_pose.pose.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_angle = yaw
        self.update_landmarks_with_odometry()

    def calculate_distance_orientation(self, landmark):
        # Calculate distance and orientation relative to the robot's current pose
        dx = landmark.x - self.current_pose.pose.position.x
        dy = landmark.y - self.current_pose.pose.position.y
        distance = np.sqrt(dx**2 + dy**2)
        orientation = np.arctan2(dy, dx) - self.current_angle
        return distance, orientation

    def transform_landmark_to_map(self, landmark):
        try:
            # Get the transformation from base_link (or your robot base frame) to map
            transform = self.tf_buffer.lookup_transform('map', 'base_link', self.get_clock().now())
            
            # Create a PoseStamped for the landmark in the robot's frame
            landmark_pose = PoseStamped()
            landmark_pose.header.frame_id = 'base_link'
            landmark_pose.pose.position.x = landmark.x
            landmark_pose.pose.position.y = landmark.y
            landmark_pose.pose.position.z = 0.0
            landmark_pose.pose.orientation.w = 1.0  # Neutral orientation

            # Transform the landmark pose to the map frame
            transformed_pose = do_transform_pose(landmark_pose, transform)
            return transformed_pose

        except Exception as e:
            self.get_logger().warn(f"Could not transform landmark {landmark.id} to map frame: {e}")
            return None
        
    def update_landmarks_with_odometry(self):
        # Update landmark positions based on current odometry
        for landmark_id, (distance, orientation) in self.landmarks.items():
            # Update landmark position relative to the new robot position
            dx = distance * np.cos(orientation + self.current_angle)
            dy = distance * np.sin(orientation + self.current_angle)
            self.landmarks[landmark_id] = (dx, dy)

    def publish_processed_landmark(self, landmark_id, distance, orientation):
        processed_landmark = Landmark()
        processed_landmark.id = landmark_id
        processed_landmark.x = distance
        processed_landmark.y = orientation
        self.processed_landmark_pub.publish(processed_landmark)
        self.get_logger().info(f'Processed Landmark ID: {landmark_id}, Distance: {distance}, Orientation: {orientation}')

def main(args=None):
    rclpy.init(args=args)
    landmark_detector = LandmarkDetector()
    rclpy.spin(landmark_detector)
    landmark_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()