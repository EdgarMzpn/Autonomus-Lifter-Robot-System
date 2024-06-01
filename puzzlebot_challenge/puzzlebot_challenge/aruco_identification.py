import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from puzzlebot_msgs.msg import Arucoinfo, ArucoArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRCodeTracker(Node):
    def __init__(self):
        super().__init__('aruco_identification')
        self.y = 0
        self.x = 0
        self.height = 0
        self.width = 0
        self.cv_bridge = CvBridge()
        self.sensor_width_mm = 3.674  # Sensor width in mm
        self.image_width_pixels = 3280  # Image width in pixels
        self.focal_length_mm = 3.04  # Lens focal length in mm
        self.object_width_real = 0.04
        self.intrinsics = None
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.subscription_info = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.qr_pub = self.create_publisher(ArucoArray, '/aruco_info', 1)

    def camera_info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        } 
        self.intrinsics = {
            'fx': 1284.144242,
            'fy': 1290.880880,
            'cx': 598.847438,
            'cy': 351.188314
        } 
        self.intrinsics = {
            'fx': 1294.852001,
            'fy': 1299.050763,
            'cx': 627.413749,
            'cy': 373.197113
        } 

    def image_callback(self, msg):
        if self.intrinsics is None:
            return
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.width = cv_image.shape[1]
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()
        # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        # arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
        

        aruco_array = ArucoArray()
        aruco_array.aruco_array = []
        if corners:
            aruco_array.length = len(corners)
             # Intrinsic parameters
            fx = self.intrinsics['fx']
            fy = self.intrinsics['fy']
            cx = self.intrinsics['cx']
            cy = self.intrinsics['cy']
            for i, aruco in enumerate(corners):

                # Calculate distances from each point to the origin
                distances = [np.linalg.norm(point) for point in aruco[0]]

                # Find the index of the closest point
                min_index = np.argmin(distances)

                # Reorder the array so the closest point is first
                sorted_corners = np.roll(aruco[0], -min_index, axis=0)

                x = int(sum(corner[0] for corner in sorted_corners) / len(sorted_corners))
                y = int(sum(corner[1] for corner in sorted_corners) / len(sorted_corners))
                w = abs(sorted_corners[0][0] - sorted_corners[1][0])
                h = abs(sorted_corners[0][1] - sorted_corners[3][1])

                z_3d = (fx * self.object_width_real) / h

                x_3d = -(x - cx) * z_3d / fx
                y_3d = (y - cy) * z_3d / fy

                # focal_length_pixels = self.focal_length_mm * (self.image_width_pixels / self.sensor_width_mm)
                # depth = (focal_length_pixels * self.object_width_real) / w
                offset = int(self.width/2 - x)
                cv2.aruco.drawDetectedMarkers(cv_image, corners)
                coords = str(np.round(x_3d, 2)) + ", " + str(np.round(y_3d, 2)) + ", " + str(np.round(z_3d, 2))
                cv2.putText(cv_image, coords, (x , y ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                aruco_info = Arucoinfo()
                aruco_info.tag = str(i)
                aruco_info.id = str(ids[i][0])
                aruco_info.point.header.frame_id = 'puzzlebot'
                aruco_info.point.point.x = x_3d
                aruco_info.point.point.y = y_3d
                aruco_info.point.point.z = z_3d
                aruco_info.offset = offset
                aruco_info.height = float(h)
                aruco_info.width = float(w)
                aruco_info.corners = []
                aruco_info.corners.append(Point(x=float(sorted_corners[0][0]), y = float(sorted_corners[0][1])))
                aruco_info.corners.append(Point(x=float(sorted_corners[1][0]), y = float(sorted_corners[1][1])))
                aruco_info.corners.append(Point(x=float(sorted_corners[2][0]), y = float(sorted_corners[2][1])))
                aruco_info.corners.append(Point(x=float(sorted_corners[3][0]), y = float(sorted_corners[3][1])))
                aruco_array.aruco_array.append(aruco_info)
        else:
            aruco_array.length = 0        
        self.qr_pub.publish(aruco_array)

        # cv2.imshow("Aruco Tracking", cv_image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    qr_code_tracker = QRCodeTracker()
    rclpy.spin(qr_code_tracker)
    qr_code_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
