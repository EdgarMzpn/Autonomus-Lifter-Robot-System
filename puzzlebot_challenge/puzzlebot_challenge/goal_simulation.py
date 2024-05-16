import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class goal_simulation(Node):
    def __init__(self):
        super().__init__('goal_simulation')
        self.pose_pub = self.create_publisher(Pose, 'goal', 1)
        while True:
            self.publish_pose()  # Publica la pose al inicio

    def publish_pose(self):
        # Define la pose que deseas publicar (ejemplo: x=1, y=2, z=0)
        pose = Pose()
        pose.position.x = 2.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    goal_sim_node = goal_simulation()
    rclpy.spin(goal_sim_node)
    goal_sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
