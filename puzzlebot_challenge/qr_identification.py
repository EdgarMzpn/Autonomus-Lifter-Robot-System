import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,Float32
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode

class QRCodeTracker(Node):
    def __init__(self):
        super().__init__('qr_code_tracker')
        self.y = 0
        self.x = 0
        self.height = 0
        self.width = 0
        self.cv_bridge = CvBridge()
        self.sensor_width_mm = 3.674  # Sensor width in mm
        self.image_width_pixels = 3280  # Image width in pixels
        self.focal_length_mm = 3.04  # Lens focal length in mm
        self.object_width_real = 0.05
        self.subscription = self.create_subscription(Image, '/video/image_raw', self.image_callback, 10)
        self.depth_pub = self.create_publisher(Float32, '/obj_depth', 1)
        self.off_center_pub = self.create_publisher(Int32, '/obj_off_center', 1)

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.width = cv_image.shape[1]
        qr_codes = decode(cv_image)
        if qr_codes:
            for qr_code in qr_codes:
                data = qr_code.data.decode('utf-8')
                (x, y, w, h) = qr_code.rect
                focal_length_pixels = self.focal_length_mm * (self.image_width_pixels / self.sensor_width_mm)
                depth = (focal_length_pixels * self.object_width_real) / w
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, str(depth), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.depth_pub.publish(Float32(data = depth))
                self.off_center_pub.publish(Int32(data = int(self.width/2 - (x+w/2))))
                # self.get_logger().info(f"Detected QR code with data: {data}, at position: ({x}, {y})")
        

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
