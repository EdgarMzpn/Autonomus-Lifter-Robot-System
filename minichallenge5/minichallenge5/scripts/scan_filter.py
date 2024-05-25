#!/usr/bin/env python3

import rospy
from numpy import linspace, inf
from math import sin
from sensor_msgs.msg import LaserScan

class ScanFilter:
	"""
	A class that implements a LaserScan filter that removes all of the points
	that are not in front of the robot.
	"""
	def _init_(self):
		self.width = 1.0
		self.extent = self.width / 2.0
		self.sub = rospy.Subscriber('puzzlebot_1/scan', LaserScan, self.callback)
		self.pub = rospy.Publisher('filtered_scan', LaserScan, queue_size=10)
		rospy.loginfo("Publishing the filtered_scan topic. Use RViz to visualize.")

	def callback(self,msg):
		"""
		Callback function to deal with incoming LaserScan messages.
		:param self: The self reference.
		:param msg: The subscribed LaserScan message.

		:publishes msg: updated LaserScan message.
		"""
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
		points = [r * sin(theta) if (theta < -2.0 or theta > 2.0) else inf for r,theta in zip(msg.ranges, angles)]
		new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]
		msg.ranges = new_ranges
		self.pub.publish(msg)
		# Find minimum and maximum range values
		min_range = min(msg.ranges)
		max_range = max(msg.ranges)

		self.get_logger().info("Minimum range: {} meters, Maximum range: {} meters".format(min_range, max_range))

if __name__ == '__main__':
	rospy.init_node('scan_filter')
	ScanFilter()
	rospy.spin()