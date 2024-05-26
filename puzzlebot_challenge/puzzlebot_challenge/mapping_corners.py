import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from puzzlebot_msgs.msg import Landmark, LandmarkList

class CornerDetector(Node):

    def __init__(self):
        super().__init__('corner_detector')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.landmark_publisher = self.create_publisher(LandmarkList, '/detected_landmarks', 10)
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg
        self.detect_corners()

    def detect_corners(self):
        if self.map_data is None:
            return
        
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        map_origin = self.map_data.info.origin.position

        grid = np.array(self.map_data.data).reshape((height, width))

        # Convert occupancy grid to binary image
        binary_map = np.where(grid > 50, 255, 0).astype(np.uint8)

        # Use OpenCV to detect corners
        corners = cv2.goodFeaturesToTrack(binary_map, maxCorners=4, qualityLevel=0.01, minDistance=10)
        corners = np.int0(corners)

        landmark_list = LandmarkList()

        for i, corner in enumerate(corners):
            x, y = corner.ravel()
            x_world = x * resolution + map_origin.x
            y_world = y * resolution + map_origin.y

            landmark = Landmark()
            landmark.x = x_world
            landmark.y = y_world
            landmark.id = f'corner_{i}'

            landmark_list.landmarks.append(landmark)
            self.get_logger().info(f'Detected corner at ({x_world}, {y_world})')

        self.landmark_publisher.publish(landmark_list)

def main(args=None):
    rclpy.init(args=args)
    node = CornerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
