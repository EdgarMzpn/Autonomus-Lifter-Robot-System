#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Joint_State_tf(Node):
    def __init__(self):
        super().__init__('Joint_State_tf')

        # Publisher 
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Transform broadcaster
        self.br = TransformBroadcaster(self)

        # Subscriber
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 1)

        # # Start the timer now
        # self.start_time = self.get_clock().now()
        # time_period = 0.1
        # self.timer = self.create_timer(time_period, self.odometry_callback)

    def odometry_callback(self, msg):
        # Create a JointState message
        self.joint_state = JointState()
        self.joint_state.header = msg.header  # Use the same timestamp and frame
        self.joint_state.name = ['wheel_coupler_joint', 'wheel_coupler_joint_2']
        # These values would ideally be calculated based on the robot's specific kinematics
        self.joint_state.position = [0., 0.]  # Placeholder values
        self.joint_state.velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        self.joint_state.effort = []

        # Publish joint states
        self.joint_pub.publish(self.joint_state)

        # Publish transform using tf2 (from 'odom' to 'base_link')
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        
        self.br.sendTransform(t)
    
def main(args= None):
    rclpy.init(args=args)
    joint_tf = Joint_State_tf()
    rclpy.spin(joint_tf)
    joint_tf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

   