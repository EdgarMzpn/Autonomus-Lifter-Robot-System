import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class Goal_sim(Node):
    def __init__(self):
        super().__init__('Goal_sim')
        self.pose_pub = self.create_publisher(Pose, 'goal', 1)
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
    goal_sim_node = Goal_sim()
    rclpy.spin(goal_sim_node)
    goal_sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
