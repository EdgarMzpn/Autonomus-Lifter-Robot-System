#!/usr/bin/env python3

# Every python controller needs these lines
import rclpy
import cv2
import argparse
from rclpy.node import Node


class Camera(Node):
	def __init__(self):
		# Initialize a ROS node named camera
		super().__init__('camera')
        
        #Method to open the camera, permits the setting up parameters
		parser = argparse.ArgumentParser()
		parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
		args = parser.parse_args()
		self.cam = cv2.VideoCapture(args.camera)

def main(args=None):
	rclpy.init(args=args)
	camera = Camera()
	rclpy.spin(camera)
	camera.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()