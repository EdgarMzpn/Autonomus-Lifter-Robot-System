import rclpy
from rclpy.node import Node
import numpy as np
import time
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
        self.handle_run_sub = self.create_subscription(Bool, '/handle_run', self.align_to_aruco, 1)

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

        self.angular_kp = 0.05
        self.angular_ki = 0.0
        self.angular_kd = 0.02

        self.integral = 0.0

        self.aruco_handled = Bool()
        self.aruco_handled.data = False
        self.aligned = False
        self.handle_instruction = Int32()

        # Aruco data
        self.aruco_array = ArucoArray()
        
        self.left_aruco = Arucoinfo()
        self.right_aruco = Arucoinfo()

        # Goal data
        self.a_goal = Arucoinfo()
        self.b_goal = Arucoinfo()
        self.c_goal = Arucoinfo()

        self.a_goal_init_offset = 0.0
        self.b_goal_init_offset = 0.0
        self.c_goal_init_offset = 0.0

        # Puzzlebot data
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_angle = 0.0

        self.start_time= self.get_clock().now()

    ##############################
    # Callback Functions
    ##############################

    def aruco_callback(self, msg: ArucoArray):
        self.aruco_array = msg

        if msg.length > 1:
            for index in range (0, msg.length):
                # Aruco data
                if msg.aruco_array[index].id == '30':
                    self.left_aruco = msg.aruco_array[index]
                elif msg.aruco_array[index].id == '31':
                    self.right_aruco = msg.aruco_array[index]
                # Goal data
                elif msg.aruco_array[index].id == '1':
                    self.a_goal = msg.aruco_array[index]
                elif msg.aruco_array[index].id == '8':
                    self.b_goal = msg.aruco_array[index]
                elif msg.aruco_array[index].id == '3':
                    self.c_goal = msg.aruco_array[index]

    def handle_callback(self, msg):
        self.handle_instruction = msg
    
    def odom_callback(self, msg: Odometry):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y

        # Get current angle from quaternion
        quaternion = msg.pose.pose.orientation
        self.current_angle = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

    ##############################
    # Handling Object
    ##############################

    def align_to_aruco(self, msg):
        if not self.aligned:
            self.aligned = self.velocity_control()
        elif self.aligned and not self.aruco_handled.data:
            self.handle_aruco()
            self.aligned = False

    def handle_aruco(self):
        drop_off = Float32()
        pick_up = Float32()

        # Angles at which servo needs to turn to execute action
        drop_off.data = 0.0
        pick_up.data = 70.0

        if self.handle_instruction.data == 0:
            self.pick_or_drop_pub.publish(pick_up)
        elif self.handle_instruction.data == 1:
            self.pick_or_drop_pub.publish(drop_off)
            self.go_backwards(0.1)
            time.sleep(0.5)
            self.go_stop()

        self.aruco_handled.data = True
        self.handled_pub.publish(self.aruco_handled)
        self.aruco_handled.data = False

    ##############################
    # Velocity Control
    ##############################

    def go_fordward(self, speed):
        self.output_velocity.linear.x = speed
        self.output_velocity.angular.z = 0.0
        self.cmd_vel_pub.publish(self.output_velocity)

        return True
    
    def go_backwards(self, speed):
        self.output_velocity.linear.x = -speed
        self.output_velocity.angular.z = 0.0
        self.cmd_vel_pub.publish(self.output_velocity)

        return True
    
    def go_stop(self):
        self.output_velocity.linear.x = 0.0
        self.output_velocity.angular.z = 0.0
        self.cmd_vel_pub.publish(self.output_velocity)

        return True

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

    def resultant_error(self, desired_position_x, desired_position_y):
        x_error = desired_position_x - self.current_position_x
        y_error = desired_position_y - self.current_position_y

        offset_scaling = 0.001

        if self.handle_instruction.data == 0:
            self.angle_error = self.left_aruco.offset * offset_scaling + self.right_aruco.offset * offset_scaling
        else:
            self.angle_error = 0.0

        self.total_position_error = np.sqrt(x_error**2 + y_error**2)

    def velocity_control(self):
        # Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9

        # Calculate resultant error
        self.resultant_error(self.current_position_x + 0.1, self.current_position_y + 0.1)

        # Adjust current pose
        self.output_position, self.prev_position_error = self.PID(self.total_position_error, self.prev_position_error, self.linear_kp, self.linear_ki, self.linear_kd)
        self.output_angle, self.prev_angle_error = self.PID(self.angle_error, self.prev_angle_error, self.angular_kp, self.angular_ki, self.angular_kd)

        self.output_velocity.linear.x = self.output_position
        self.output_velocity.angular.z = self.output_angle
        self.get_logger().info(f'instruction_state: {self.handle_instruction.data}')

        # self.get_logger().info(f'Velocity Control: linear={self.output_velocity.linear.x} angular={self.output_velocity.angular.z}')

        if self.handle_instruction.data == 0:
            if self.aruco_array.length == 0:
                self.go_fordward(0.1)
                time.sleep(1.8)
                self.go_stop()

                return True
            
            else:
                self.cmd_vel_pub.publish(self.output_velocity)

        elif self.handle_instruction.data == 1:
            if self.aruco_array.length > 0:
                for index in range(0, self.aruco_array.length):
                    if self.aruco_array[index].id == self.a_goal.id:
                        self.go_fordward(0.1)
                        time.sleep(0.5)
                        self.go_stop()

                return True
            
            else:
                self.cmd_vel_pub.publish(self.output_velocity)

def main(args=None):
    rclpy.init(args=args)
    odometry = ObjectHandler()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
