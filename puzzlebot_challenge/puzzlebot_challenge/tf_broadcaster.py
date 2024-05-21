from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations


# This node publishes three child static frames in reference to their parent frames as below:
# parent -> link_mast            child -> fk_link_mast
# Tf frames are not defined according to the DH convention by default 
# The new frames have been declared to make the computation of 
# forward and inverse kinematics easier by defining the new frames
# according to the DH convention
class FixedFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('stretch_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        time_period = 0.1 # seconds
        self.timer = self.create_timer(time_period, self.broadcast_timer_callback)

        self.base = TransformStamped()
        self.base.header.frame_id = 'odom'
        self.base.child_frame_id = 'base_link'
        self.base.transform.translation.x = 0.0
        self.base.transform.translation.y = 0.0
        self.base.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.base.transform.rotation.x = q[0]
        self.base.transform.rotation.y = q[1]
        self.base.transform.rotation.z = q[2]
        self.base.transform.rotation.w = q[3]

        self.base_footprint = TransformStamped()
        self.base_footprint.header.frame_id = 'base_link'
        self.base_footprint.child_frame_id = "base_footprint"
        self.base_footprint.transform.translation.x = 0.0
        self.base_footprint.transform.translation.y = 0.0
        self.base_footprint.transform.translation.z = 0.0
        self.base_footprint.transform.rotation.x = 0.0
        self.base_footprint.transform.rotation.y = 0.0
        self.base_footprint.transform.rotation.z = 0.0
        self.base_footprint.transform.rotation.w = 1.0

        self.get_logger().info("Publishing Tf frames. Use RViz to visualize.")

    def broadcast_timer_callback(self):
        self.base.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.base)
        self.br.sendTransform(self.base_footprint)


def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = FixedFrameBroadcaster()

    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()