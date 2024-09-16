import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from puzzlebot_msgs.msg import Arucoinfo, ArucoArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class ArUco_tracker(Node):
    """
    A ROS2 node that tracks ArUco markers in video frames.

    This class subscribes to image and camera info topics, detects ArUco markers in the images,
    computes their positions in 3D space, and publishes the information.

    Params: 
        display (bool): If true, the detected markers and their coordinates are displayed in a window
    """
    
    def __init__(self):
        """
        Initializes the ArUcoTracker node.

        The 'display' parameter is declared and read from the parameter server.
        """
        super().__init__('aruco_identification')

        # Parameters handling
        self.declare_parameter('display', False)
        self.display = self.get_parameter('display').get_parameter_value().bool_value

        # Initialize variables
        self.cv_bridge = CvBridge()
        self.width = 0 # Image width
        self.object_real_width = 0.04 # Measured marker real width

        # Initialize subscribers for camera info and camera image
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.subscription_info = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # Initialize publisher for ArUco array with marker information
        self.qr_pub = self.create_publisher(ArucoArray, '/aruco_info', 1)

    def camera_info_callback(self, msg):
        """
        Callback function for the camera info topic.

        Extracts the intrinsic camera parameters from the CameraInfo message.

        Args:
            msg (CameraInfo): The camera info message.
        """
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        } 
        


    def image_callback(self, msg):
        """
        Callback function for the image topic.

        Converts the image message to an OpenCV image, detects ArUco markers,
        computes their 3D positions, and publishes the information.

        Args:
            msg (Image): The image message
        """
        if self.intrinsics is None:
            return
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.width = cv_image.shape[1]
        
        # The commands may vary depending on the openCV version used
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()

        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
        

        aruco_array = ArucoArray() #Initialize empy ArucoArray
        aruco_array.aruco_array = []

        if corners:
            aruco_array.length = len(corners) # Assign to the array length the number of markers

            # Intrinsic parameters
            fx = self.intrinsics['fx']
            fy = self.intrinsics['fy']
            cx = self.intrinsics['cx']
            cy = self.intrinsics['cy']

            for i, aruco in enumerate(corners):

                distances = [np.linalg.norm(point) for point in aruco[0]] # Calculate distances from each point to the origin
                min_index = np.argmin(distances) # Find the index of the closest point      
                sorted_corners = np.roll(aruco[0], -min_index, axis=0) # Reorder the array so the closest point is first

                # Compute marker position and size
                x = int(sum(corner[0] for corner in sorted_corners) / len(sorted_corners))
                y = int(sum(corner[1] for corner in sorted_corners) / len(sorted_corners))
                w = abs(sorted_corners[0][0] - sorted_corners[1][0])
                h = abs(sorted_corners[0][1] - sorted_corners[3][1])

                # Compute 3D puzzlebot frame coordinates
                z_3d = (fx * self.object_real_width) / h
                x_3d = -(x - cx) * z_3d / fx
                y_3d = (y - cy) * z_3d / fy
                
                offset = int(self.width/2 - x) # Compute marker position offset from the image center
                
                if self.display:
                    cv2.aruco.drawDetectedMarkers(cv_image, corners)
                    coords = str(np.round(x_3d, 2)) + ", " + str(np.round(y_3d, 2)) + ", " + str(np.round(z_3d, 2))
                    cv2.putText(cv_image, coords, (x , y ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Fill message
                aruco_info = Arucoinfo()
                aruco_info.tag = str(i)
                aruco_info.id = str(ids[i][0])
                aruco_info.point.header.frame_id = 'puzzlebot'
                aruco_info.point.point.x = x_3d
                aruco_info.point.point.y = y_3d
                aruco_info.point.point.z = z_3d - 0.12
                aruco_info.offset = offset
                aruco_info.height = float(h)
                aruco_info.width = float(w)
                aruco_array.aruco_array.append(aruco_info)
        else:
            aruco_array.length = 0
                   
        self.qr_pub.publish(aruco_array) # Publish array wheter empty or not

        if self.display:
            cv2.imshow("Aruco Tracking", cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_tracker = ArUco_tracker()
    rclpy.spin(aruco_tracker)
    aruco_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
