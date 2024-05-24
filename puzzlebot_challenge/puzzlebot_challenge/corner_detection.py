import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import Point

class CornerDetectionNode(Node):
    def __init__(self):
        super().__init__('corner_detection_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Point, '/corners', 10)

    def scan_callback(self, msg):
        # self.get_logger().info("Entering the scan_cb")
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter out invalid range readings
        valid_indices = np.where((ranges > msg.range_min) & (ranges < msg.range_max))
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Convert to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = np.vstack((x, y)).T

        # Detect lines
        lines = self.detect_lines(points)

        # Find intersections
        intersections = self.find_intersections(lines)
        self.publish_corners(intersections)

    def detect_lines(self, points, distance_threshold=0.1, angle_threshold=np.pi / 36):
        # self.get_logger().info("Entering the detect_lines")
        lines = []
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            line_found = False
            for line in lines:
                if self.point_to_line_distance(p1, line) < distance_threshold and self.angle_between_lines(p1, p2, line) < angle_threshold:
                    line.append(p2)
                    line_found = True
                    break
            if not line_found:
                lines.append([p1, p2])
        return lines

    def point_to_line_distance(self, point, line):
        # self.get_logger().info("Entering the point_to_line")
        p1, p2 = line[0], line[-1]
        return np.abs(np.cross(p2 - p1, point - p1)) / np.linalg.norm(p2 - p1)

    def angle_between_lines(self, p1, p2, line):
        line_vec = line[-1] - line[0]
        new_line_vec = p2 - p1
        cos_angle = np.dot(line_vec, new_line_vec) / (np.linalg.norm(line_vec) * np.linalg.norm(new_line_vec))
        return np.arccos(cos_angle)

    def find_intersections(self, lines):
        # self.get_logger().info("Entering the find intersections")
        intersections = []
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line1, line2 = lines[i], lines[j]
                intersection = self.calculate_intersection(line1, line2)
                if intersection is not None:
                    intersections.append(intersection)
        return intersections

    def calculate_intersection(self, line1, line2):
        # self.get_logger().info("Entering the calculation")
        x1, y1 = line1[0]
        x2, y2 = line1[-1]
        x3, y3 = line2[0]
        x4, y4 = line2[-1]

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        xdiff = (x1 - x2, x3 - x4)
        ydiff = (y1 - y2, y3 - y4)

        div = det(xdiff, ydiff)
        if div == 0:
            return None  # Lines do not intersect

        d = (det((x1, y1), (x2, y2)), det((x3, y3), (x4, y4)))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return np.array([x, y])

    def publish_corners(self, intersections):
        # self.get_logger().info("Entering the publisher")
        for (x, y) in intersections:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            self.publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = CornerDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
