#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Joint_State_tf:
    def __init__(self):
        rospy.init_node('Joint_State_tf')


        #Publisher
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size = 10)

        #Transform broadcaster
        self.br = TransformBroadcaster()

        #Subscriber
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odometry_callback)

    def odometry_callback(self, msg):
        joint_state = JointState()
        joint_state_header = msg.header
        joint_state.name = ['wheel_coupler_joint', 'wheel_coupler_joint_2']

        joint_state.position = [0.,0.]
        joint_state.velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        joint_state.effort = []


        #Publish joint states
        self.joint_pub.publish(joint_state)


        # Publish transform using tf2 (from 'odom' to 'base_link')
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

def main(args=None):
    joint_tf = Joint_State_tf()
    rospy.spin()

if __name__ == '__main__':
    main()




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





