import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from puzzlebot_msgs.msg import ArucoArray, Arucoinfo
from std_msgs.msg import Int32, Float32, Bool
from tf_transformations import euler_from_quaternion

class ObjectHandler(Node):
    def __init__(self):
        super().__init__('handle_object')

        # Subscribers
        self.aruco_sub = self.create_subscription(ArucoArray, '/aruco_info', self.aruco_callback, 10)
        self.handle_sub = self.create_subscription(Int32, '/handle', self.handle_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.handled_pub = self.create_publisher(Bool, '/handled_aruco', 1)
        self.pick_or_drop_pub = self.create_publisher(Float32, '/ServoAngle', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Control variables
        self.total_position_error = 0.0
        self.angle_error = 0.0

        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0

        self.output_position = 0.0
        self.output_angle = 0.0

        self.output_velocity = Twist()
        self.output_error = Point()

        self.linear_kp = 0.20
        self.linear_ki = 0.0
        self.linear_kd = 0.05

        self.angular_kp = 0.48
        self.angular_ki = 0.0
        self.angular_kd = 0.02

        self.integral = 0.0

        self.aligned = False
        self.aruco_handled = Bool()

        # Aruco data
        self.aruco = Arucoinfo()
        self.aruco_position_x = 0.0
        self.aruco_position_y = 0.0

        # Target data
        self.target = Point()
        self.target.x = 0.0
        self.target.y = 0.0
        self.target.z = self.aruco.point.point.z
        self.target_angle = 0.0

        # Puzzlebot data
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_angle = 0.0

        # Start the timer now
        self.start_time = self.get_clock().now()
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.align_to_aruco)

    ##############################
    # Callback Functions
    ##############################

    def aruco_callback(self, msg: ArucoArray):
        if msg.length > 0:
            self.aruco = msg.aruco_array[0]
            if self.aruco_position_x == 0.0 and self.aruco_position_y == 0.0:
                self.transform_cube_position(self.aruco.point.point)
                self.target_position()
                self.transform_cube_position(self.target)

    def handle_callback(self, msg: Int32):
        self.handle = msg.data
    
    def odom_callback(self, msg: Odometry):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y

        # Get current angle from quaternion
        quaternion = msg.pose.pose.orientation
        self.current_angle = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

    ##############################
    # Handling Object
    ##############################

    def align_to_aruco(self):
        # target.x, target.y, target_angle = self.target_position

        if not self.aligned:
            self.aligned = self.velocity_control(self.target.x, self.target.y, self.target_angle)
        elif self.aligned:
            self.handle_aruco()

        self.get_logger().info(f'Target: x={self.target.x} y={self.target.y}')
        
        self.handled_pub.publish(self.aruco_handled)

    def handle_aruco(self):
        # Angles at which servo needs to turn to execute action
        drop_off = 0.0
        pick_up = 70.0

        arrived = self.velocity_control(self.aruco_position_x, self.aruco_position_y, self.current_angle)

        if arrived and not self.aruco_handled.data:
            if self.handle == 0:
                self.pick_or_drop_pub.publish(pick_up)
                self.pick_or_drop_pub.publish(pick_up - 5)
            elif self.handle == 1:
                self.pick_or_drop_pub.publish(drop_off)
            
            # Step back
            self.aruco_handled.data = self.velocity_control(self.aruco_position_x - 1, self.aruco_position_y - 1, self.current_angle)

    ##############################
    # Target Data
    ##############################

    def target_position(self):
        left_bottom_corner_index = 2
        right_bottom_corner_index = 3

        left_bottom_corner_position = self.aruco.corners[left_bottom_corner_index]
        right_bottom_corner_position = self.aruco.corners[right_bottom_corner_index]

        left_to_right_corner_length = np.sqrt((left_bottom_corner_position.x -right_bottom_corner_position.x)**2 + (left_bottom_corner_position.y - right_bottom_corner_position.y)**2)
        
        aux_cathetus_length = np.abs(np.abs(left_bottom_corner_position.x) - np.abs(right_bottom_corner_position.x))

        # Generate offset point
        # Direction vector
        vector_x = left_bottom_corner_position.x - right_bottom_corner_position.x
        vector_y = left_bottom_corner_position.y - right_bottom_corner_position.y

        # Normalized Perpendicual vector
        normal_vector_x = -vector_x / left_to_right_corner_length
        normal_vector_y = vector_y / left_to_right_corner_length

        # Paralel line
        parallel_offset = 0.5
        parallel_x1 = left_bottom_corner_position.x + parallel_offset * -normal_vector_x
        parallel_y1 = left_bottom_corner_position.y + parallel_offset * -normal_vector_y
        parallel_x2 = right_bottom_corner_position.x + parallel_offset * -normal_vector_x
        parallel_y2 = right_bottom_corner_position.y + parallel_offset * -normal_vector_y

        # Target
        self.target.x = (parallel_x1 + parallel_x2) / 2
        self.target.y = (parallel_y1 + parallel_y2) / 2
        self.target_angle = np.arccos(aux_cathetus_length / left_to_right_corner_length) + self.current_angle
        # theta_deg = np.degrees(theta_rad)

    def transform_cube_position(self, aruco_point):
        rotation_matrix = np.array([
                    [np.cos(self.current_angle), -np.sin(self.current_angle)],
                    [np.sin(self.current_angle), np.cos(self.current_angle)]
                ])

        # Camera coordinates to robot coordinates
        puzzlebot_coords = np.array([aruco_point.z, aruco_point.x])
        world_coords = rotation_matrix.dot(puzzlebot_coords)

        # Robot coordinates to odometry coordinates
        self.aruco_position_x = world_coords[0] + self.current_position_x
        self.aruco_position_y = world_coords[1] + self.current_position_y

    ##############################
    # Velocity Control
    ##############################

    def PID(self, error, prev_error, kp, ki, kd):
        # Proportional term
        P = kp * error
        
        # Integral term
        self.integral += error
        I = ki * self.integral
        
        # Derivative term
        derivative = error - prev_error
        D = kd * derivative
        
        # Compute PID output
        output = P + I + D
        
        return output, error

    def resultant_error(self, desired_position_x, desired_position_y, desired_angle):
        x_error = desired_position_x - self.current_position_x
        y_error = desired_position_y - self.current_position_y

        self.total_position_error = np.sqrt(x_error**2 + y_error**2)

        # desired_angle = np.arctan2(y_error, x_error)
        self.angle_error = desired_angle - self.current_angle
        
        # Normalize angle to be within [-π, π]
        self.angle_error = np.arctan2(np.sin(self.angle_error), np.cos(self.angle_error))

    def velocity_control(self, desired_position_x, desired_position_y, desired_angle):
        #Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9

        # Calculate resultant error
        self.resultant_error(desired_position_x, desired_position_y, desired_angle)

        # Adjust current pose
        self.output_position, self.prev_position_error = self.PID(self.total_position_error, self.prev_position_error, self.linear_kp, self.linear_ki, self.linear_kd)
        self.output_angle, self.prev_angle_error = self.PID(self.angle_error, self.prev_angle_error, self.angular_kp, self.angular_ki, self.angular_kd)
        
        if abs(self.prev_angle_error) > 0.1:
            self.output_velocity.angular.z = self.output_angle
            self.output_velocity.linear.x = 0.0
        else:
            self.output_velocity.linear.x = self.output_position
            self.output_velocity.angular.z = 0.0

        self.output_error.x = self.total_position_error
        self.output_error.y = self.prev_angle_error

        self.cmd_vel_pub.publish(self.output_velocity)

        tolerance = 0.2

        if self.total_position_error < tolerance and self.total_position_error > -tolerance:
            arrive = True
        else:
            arrive= False

        return arrive

def main(args=None):
    rclpy.init(args=args)
    odometry = ObjectHandler()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()