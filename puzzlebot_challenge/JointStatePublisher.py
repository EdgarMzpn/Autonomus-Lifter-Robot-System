import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class JSP(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.odometry_subscriber_ = self.create_subscription(TransformStamped, 'odometry_transform', self.odometry_callback, 10)
        self.joint_states = JointState()

        # Add the names of your robot's joints
        self.joint_states.name = ['chassis', 'wheel_coupler', 'wheel_coupler_2']  # Example joint names
        self.get_logger().info("Joint State Publisher node has been initiated.")

    def odometry_callback(self, msg):
        # Assuming your odometry transform provides the rotation angles for the joints
        # Modify this logic according to your actual implementation
        q = quaternion_from_euler(0., 0., self.angle)
        joint_values = [q]
        self.publish_joint_states(joint_values)

    def publish_joint_states(self, joint_values):
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position = joint_values
        self.publisher_.publish(self.joint_states)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JSP()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
