import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32,Float32
from puzzlebot_msgs.msg import Arucoinfo
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
print(cv2.__version__)

class QRCodeTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.y = 0
        self.x = 0
        self.height = 0
        self.width = 0
        self.cv_bridge = CvBridge()
        self.sensor_width_mm = 3.674  # Sensor width in mm
        self.image_width_pixels = 3280  # Image width in pixels
        self.focal_length_mm = 3.04  # Lens focal length in mm
        self.object_width_real = 0.05
        self.intrinsics = None
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.subscription_info = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.qr_pub = self.create_publisher(Arucoinfo, '/aruco_info', 1)

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

        if corners:
             # Intrinsic parameters
            fx = self.intrinsics['fx']
            fy = self.intrinsics['fy']
            cx = self.intrinsics['cx']
            cy = self.intrinsics['cy']
            for i, aruco in enumerate(corners):
                x = int(sum(corner[0][0] for corner in aruco) / len(aruco))
                y = int(sum(corner[0][1] for corner in aruco) / len(aruco))
                w = abs(aruco[0][0][0] - aruco[0][1][0])
                h = abs(aruco[0][0][1] - aruco[0][3][1])

                z_3d = (fx * self.object_width_real) / h

                x_3d = (x - cx) * z_3d / fx
                y_3d = (y - cy) * z_3d / fy

                # focal_length_pixels = self.focal_length_mm * (self.image_width_pixels / self.sensor_width_mm)
                # depth = (focal_length_pixels * self.object_width_real) / w
                offset = int(self.width/2 - (x+w/2))
                cv2.aruco.drawDetectedMarkers(cv_image, corners)
                cv2.putText(cv_image, str(z_3d), (x + int(w /2), y + int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                qr_info = Arucoinfo()
                qr_info.tag = str(i)
                qr_info.depth = z_3d
                qr_info.offset = offset
                self.qr_pub.publish(qr_info)
                
        """if qr_codes:
            for i, qr_code in enumerate(qr_codes):
                data = qr_code.data.decode('utf-8')
                (x, y, w, h) = qr_code.rect
                focal_length_pixels = self.focal_length_mm * (self.image_width_pixels / self.sensor_width_mm)
                depth = (focal_length_pixels * self.object_width_real) / w
                offset = int(self.width/2 - (x+w/2))
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, str(depth), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                qr_info = Arucoinfo()
                qr_info.qr_exists = True
                qr_info.tag = 'OneAndOnly'
                qr_info.depth = depth
                qr_info.offset = offset
                self.qr_pub.publish(qr_info)
                # self.get_logger().info(f"Detected QR code with data: {data}, at position: ({x}, {y})")"""
        

        cv2.imshow("QR Code Tracking", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    qr_code_tracker = QRCodeTracker()
    rclpy.spin(qr_code_tracker)
    qr_code_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
