import rclpy
import numpy as np
from rclpy import Node
from sensor_msgs.msg import PointCloud2

class KalmanFilter:
    def __init__(self, dt, measurement_variance):
        self.dt = dt
        self.measurement_variance = measurement_variance

        # State transition matrix (constant velocity model)
        self.F = np.array([[1, dt],
                           [0, 1]])

        # Measurement matrix
        self.H = np.array([[1, 0]])

        # Process noise covariance matrix
        self.Q = np.array([[dt**4/4, dt**3/2],
                           [dt**3/2, dt**2]])

        # Measurement noise covariance
        self.R = np.array([[measurement_variance]])

        # Initial state estimate
        self.x = np.zeros((2, 1))

        # Initial covariance matrix
        self.P = np.eye(2)

    def predict(self):
        # Predict the next state
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        # Update the state estimate based on measurement z
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(PointCloud2, '/rplidar/point_cloud', self.pc_callback, 10)
        self.kalman_filter = KalmanFilter(dt=1.0, measurement_variance=0.1)

    def pc_callback(self, msg):
        # Extract point cloud data from the message
        # Process the point cloud data to extract position information
        # For simplicity, let's assume we extract position data (x, y) of detected points
        # Replace this with your actual point cloud processing code
        points = np.array(msg.data, dtype=np.float32)
        points = points.reshape(-1, 4)
        positions = points[:, :2]

        # Perform Kalman filtering for each detected point
        for position in positions:
            # Predict step
            self.kalman_filter.predict()

            # Update step
            self.kalman_filter.update(position)

            # Get estimated position
            estimated_position = self.kalman_filter.x
            print("Estimated position:", estimated_position)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
