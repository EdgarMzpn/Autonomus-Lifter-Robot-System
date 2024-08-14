#!/usr/bin/env python3

# Every python controller needs these lines
import rclpy
from rclpy.node import Node
from numpy import linspace, inf
from math import sin
import numpy as np

# We're going to subscribe to a LaserScan message
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
	"""
	A ROS2 node that filters LiDAR data based on y-coordinates.

	This class subscribes to the scan topic, processes incoming `LaserScan` messages to filter out
	data points that are too far from the x-axis (y-coordinate exceeds a threshold), and publishes 
	the filtered data to the filtered_scan topic.
	"""
	def __init__(self):
		"""
		Initialize the filter_scan node
		
		Sets up the publisher for the filtered scan data and the subscriber 
		to the raw scan data.
		"""
		# Initialize a ROS node named scan_filter
		super().__init__('scan_filter')

		# Set up a publisher that will publish on a topic called "filtered_scan" with a LaserScan message type
		self.pub = self.create_publisher(LaserScan, '/filtered_scan', 10)

		# Set up a subscriber.  We're going to subscribe to the topic "scan", looking for LaserScan messages.
		self.sub = self.create_subscription(LaserScan, '/scan', self.scan_filter_callback, 10)
		
		# We're going to assume that the robot is pointing up the x-axis, so that any points with y coordinates further than half of the defined
		# width (1 meter) from the axis are not considered
		self.width = 1
		self.extent = self.width / 2.0

		self.get_logger().info("Publishing the filtered_scan topic. Use RViz to visualize.")

	def scan_filter_callback(self,msg):
		"""
		Callback function for the filter message topic

		Filters out data points where the y-coordinate of the scan is beyond 
		the predefined extent and republishes the filtered data.

		Args:
			msg (ScanFilter): The incoming LaserScan message containing raw scan data.
		"""
		# Figure out the angles of the scan.
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

		# Work out the y coordinates of the ranges
		points = [r * sin(theta) if (theta < -1.0 or theta > 1.0) else inf for r,theta in zip(msg.ranges, angles)]

		# If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
		new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

		# Substitute in the new ranges in the original message, and republish it
		msg.ranges = new_ranges
		self.pub.publish(msg)
		# Find minimum and maximum range values
		min_range = min(msg.ranges)
		min_index = np.argmin(msg.ranges)
		max_range = max(msg.ranges)

		# Create a logger to check the ranges in the terminal
		self.get_logger().info("Minimum range: {} meters and index: {},length: {}, Maximum range: {} meters".format(min_range, min_index, len(msg.ranges), max_range))


def main(args=None):
	rclpy.init(args=args)
	scan_filter = ScanFilter()
	rclpy.spin(scan_filter)
	scan_filter.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
