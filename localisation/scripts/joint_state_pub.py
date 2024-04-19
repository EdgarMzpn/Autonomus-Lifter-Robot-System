#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

def odometry_callback(msg):
    q = quaternion_from_euler(0, 0, angle)
    joint_values = [q]
    publisher.publish(joint_values)

def publish_joint_states(joint_values):
    joint_states.header.stamp = rospy.Time.now()
    joint_states.position = joint_values
    publisher.publish(joint_states)



def main():
    rospy.init_node('joint_state_publisher')
    publisher = rospy.Publisher('joint_states', JointState, queue_size = 10)
    rospy.Subscriber('odom', TransformStamped, odometry_callback, publisher)
    joint_states = JointState()
    rospy.spin()

    joint_states.name = ['chassis', 'wheel_coupler', 'wheel_coupler_2']
    rospy.loginfo("Joint State Publisher node has been initiated.")

if __name__=='__main__':
    main()


