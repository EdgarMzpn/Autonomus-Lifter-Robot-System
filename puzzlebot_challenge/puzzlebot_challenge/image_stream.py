#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class Image_stream(Node):
    def __init__(self):
        super().__init__('QR_Identification')
        self.publisher = self.create_publisher(Image, 'video_source/raw', 10)
        self.data_pub = self.create_publisher(CameraInfo, 'video_source/camera_info', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.new_resolution = (1280, 720)
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            resized_frame = cv2.resize(frame, self.new_resolution)
            # resized_frame = frame
            img_msg = self.bridge.cv2_to_imgmsg(resized_frame, encoding="bgr8")
            camera_info = CameraInfo()
            camera_info.header.stamp.sec = 1715820136
            camera_info.header.stamp.nanosec = 969707481
            camera_info.header.frame_id = "camera"

            camera_info.height = 720
            camera_info.width = 1280

            camera_info.distortion_model = "plumb_bob"
            camera_info.d = [-0.33888, 0.116192, 0.000114, -0.001292, 0.0]

            camera_info.k = [
                776.88108, 0.0, 662.6974,
                0.0, 777.29396, 406.37559,
                0.0, 0.0, 1.0
            ]

            camera_info.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]

            camera_info.p = [
                594.12457, 0.0, 662.3023, 0.0,
                0.0, 711.47339, 414.14309, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]

            camera_info.binning_x = 0
            camera_info.binning_y = 0

            camera_info.roi.x_offset = 0
            camera_info.roi.y_offset = 0
            camera_info.roi.height = 0
            camera_info.roi.width = 0
            camera_info.roi.do_rectify = False
            self.data_pub.publish(camera_info)
            self.publisher.publish(img_msg)
            
    def shutdown(self):
        self.cap.release()



def main(args=None):
    rclpy.init(args=args)
    img = Image_stream()
    rclpy.spin(img)
    img.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        