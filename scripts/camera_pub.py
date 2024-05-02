#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def webcam_publisher():
    # Initialize ROS node
    rospy.init_node('webcam_publisher', anonymous=True)
    
    # Initialize OpenCV video capture
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Failed to open webcam.")
        return
    
    # Initialize ROS publisher
    pub = rospy.Publisher('/video', Image, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(10)  # Adjust the publishing rate as needed
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Convert the frame to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the ROS Image message
            pub.publish(ros_image)
        rate.sleep()

    # Release OpenCV video capture and close ROS node
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass
