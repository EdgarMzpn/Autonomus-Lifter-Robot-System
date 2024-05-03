#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Image_stream(Node):
    def __init__(self):
        super().__init__('QR_Identification')
        self.publisher = self.create_publisher(Image, 'video/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        # self.new_resolution = (640, 480)
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # resized_frame = cv2.resize(frame, self.new_resolution)
            resized_frame = frame
            img_msg = self.bridge.cv2_to_imgmsg(resized_frame, encoding="bgr8")
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
        