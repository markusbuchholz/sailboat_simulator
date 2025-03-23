import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Float64MultiArray
from marvelmind_ros2_msgs.msg import HedgePosition
from geometry_msgs.msg import Twist, Vector3
import math
import time

class SimpleYawController(Node):
    def __init__(self):
        super().__init__('simple_yaw_controller')
        
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        subscriber_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()
        
        self.port_thrust_publisher = self.create_publisher(Float32, '/blueboat/send_port_motor_0_100_thrust', qos_profile=custom_qos, callback_group=self.callback_group)
        self.stbd_thrust_publisher = self.create_publisher(Float32, '/blueboat/send_stbd_motor_0_100_thrust', qos_profile=custom_qos, callback_group=self.callback_group)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, "/blueboat/cmd_vel", qos_profile=custom_qos, callback_group=self.callback_group)

        self.subscription = self.create_subscription(Float32, '/bluebot/imu_compass_fused', self.fuser_callback, custom_qos, callback_group=self.callback_group)
        self.set_yaw_subscription = self.create_subscription(Float32, '/set_yaw', self.set_yaw_callback, custom_qos, callback_group=self.callback_group)
        self.get_logger().info("Subscribed to /set_yaw")
        
        self.waypoint_subscription = self.create_subscription(
            Float64MultiArray,
            '/waypoints',
            self.waypoint_callback,
            subscriber_qos_profile,
            callback_group=self.callback_group
        )
        self.get_logger().info("Subscribed to /waypoints")
        
        self.position_subscription = self.create_subscription(
            HedgePosition,
            '/hedgehog_pos',
            self.position_callback,
            subscriber_qos_profile,
            callback_group=self.callback_group
        )
        self.get_logger().info("Subscribed to /hedgehog_pos")
        
        
        self.set_yaw_publisher = self.create_publisher(Float32, '/set_yaw', qos_profile=custom_qos, callback_group=self.callback_group)
        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/waypoints', qos_profile=custom_qos, callback_group=self.callback_group)


        # Initial yaw values
        self.current_roll = None
        self.target_yaw = None
        self.fused_yaw = None

        # PID controller gains
        self.angular_kPx = 4.0  
        self.angular_kIx = 0.0  
        self.angular_kDx = 6.0 
        
        self.linear_kP = 0.5
        self.linear_kI = 0.1
        self.linear_kD = 0.05

        self.angular_kP = 4.0  
        self.angular_kI = 0.0   
        self.angular_kD = 6.0 
        
        self.angular_kPm = 5.0  
        self.angular_kIm = 0.0  
        self.angular_kDm = 6.0

        # PID controller state
        self.integral = 0.0
        self.prev_error = 0.0
        self.linear_integral = 0.0
        self.linear_prev_error = 0.0

        # Tolerance for stopping the robot when close to the target yaw
        self.heading_tolerance = 3.0  # 5 degrees tolerance
        self.position_tolerance = 0.5  # 1.0 meter tolerance
        self.control_heading_error = 4.0  
        self.dp_error_yaw = 10.0
        self.dp_error_distance = 3.0
        
        self.marvelmind_yaw = 0.0 
        self.angle_offset = 40.0
        
        # Timer for constant checking
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.callback_group)
        self.get_logger().info("Timer started")
        
        self.current_position = None
        self.target_position = None
        self.dp_position = None
        self.rotating = False
        self.moving_to_waypoint = False
        self.dynamic_position = False
        
        self.wave_control = 1.0

    def listener_callback(self, msg):
        orientation = msg.orientation
        roll, pitch, yaw = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_roll = math.degrees(roll)  # Use roll in degrees
    
    def fuser_callback(self, msg):
        self.fused_yaw = msg.data
        self.current_roll = (self.fused_yaw + self.angle_offset) % 360
        
    def position_callback(self, msg):
        self.current_position = (msg.x_m, msg.y_m)
        self.get_logger().info(f"Current position: {self.current_position}, Current orientation: {self.current_roll} ")
        
    def waypoint_callback(self, msg):
        self.target_position = (msg.data[0], msg.data[1])
        self.get_logger().info(f"Received waypoint: {self.target_position}")
        
        if self.current_position:
            # Calculate the relative position of the waypoint from the current position
            dx = self.target_position[0] - self.current_position[0]
            dy = self.target_position[1] - self.current_position[1]
            self.get_logger().info(f"dx: {dx}, dy: {dy}")

            if dx < 0:
                self.wave_control = -1.0

            # Calculate the target yaw and apply the angle offset
            target_yaw_world = math.degrees(math.atan2(dy, dx))
            target_yaw_world = self.normalize_angle(target_yaw_world + self.marvelmind_yaw)
            self.target_yaw = self.normalize_angle(target_yaw_world)
            self.get_logger().info(f"Calculated target yaw: {self.target_yaw:.2f}, Current yaw: {self.current_roll:.2f}")
            self.rotating = True
            self.moving_to_waypoint = False
            self.dynamic_position = False
            self.get_logger().info("Starting navigation to target waypoint")

    @staticmethod
    def euler_from_quaternion(quat):
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw
        
    def set_yaw_callback(self, msg):
        self.target_yaw = msg.data  # Target yaw is in degrees
        self.get_logger().info(f"Received target yaw: {self.target_yaw:.2f} degrees")

    ############################
    ###### STATE MACHINE #######
    ############################

    def timer_callback(self):
        if self.current_roll is None:
            self.get_logger().info("Current roll not available yet.")
            return
        if self.target_yaw is None and not self.moving_to_waypoint:
            self.get_logger().info("Target yaw not available yet.")
            return

        if self.rotating:
            self.adjust_orientation()
            
        if self.dynamic_position:
            self.run_dynamic_position()
        
        if self.moving_to_waypoint:
            self.move_to_waypoint()
            

    ############################
    ############################
    
    def adjust_orientation(self):
        heading_error = self.normalize_angle(self.target_yaw - self.current_roll)
        self.get_logger().info(f"Current Roll: {self.current_roll:.2f} degrees, Target Yaw: {self.target_yaw:.2f} degrees, Heading Error: {heading_error:.2f} degrees")

        if abs(heading_error) <= self.heading_tolerance:
            self.get_logger().info("Target yaw achieved. Stopping thrusters.")
            self.publish_cmd_vel(0.0)  # Stop the robot
            self.integral = 0.0  # Reset integral term
            self.prev_error = 0.0  # Reset previous error
            self.rotating = False
            self.moving_to_waypoint = True  # Start moving to the final waypoint
            self.dynamic_position = False
            return

        # PID calculations
        self.integral += heading_error * self.timer_period
        derivative = (heading_error - self.prev_error) / self.timer_period
        control_effort = (self.angular_kPx * heading_error) + (self.angular_kIx * self.integral) + (self.angular_kDx * derivative)
        self.prev_error = heading_error

        # Limit control effort to thrust limits and apply dead zone
        control_effort = max(min(control_effort, 20.0), -20.0)
        if -7.0 < control_effort < 7.0:
            control_effort = 0.0

        self.publish_cmd_vel(control_effort)
    
    def publish_cmd_vel(self, port_thrust, stbd_thrust=None):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        
        cmd_vel_msg.linear.y = -port_thrust * 3.0  
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
    def move_to_waypoint(self):
        if self.target_position:
            dx = self.target_position[0] - self.current_position[0]
            dy = self.target_position[1] - self.current_position[1]
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)
            self.get_logger().info(f"Distance to target: {distance_to_target:.2f} meters")

            if distance_to_target < self.position_tolerance:  # Threshold distance to stop
                self.get_logger().info("Final waypoint reached. Stopping thrusters.")
                self.dp_position = self.target_position 
                self.target_position = None  # Reset target position after achieving it
                self.rotating = False
                self.moving_to_waypoint = False  # Stop moving to the waypoint
                self.dynamic_position = True  # Activate DP
                self.linear_integral = 0.0  # Reset integral term
                self.linear_prev_error = 0.0  # Reset previous error
                self.stop_asv()
                return

            self.calculate_target_yaw(self.target_position)  # LOS calculation
            heading_error = self.normalize_angle(self.target_yaw - self.current_roll)
            delta_time = self.timer_period
            linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_target, self.linear_prev_error, self.linear_integral, delta_time)
            angular_velocity = self.calculate_pid(self.angular_kPm, self.angular_kIm, self.angular_kDm, heading_error, self.prev_error, self.integral, delta_time)
            angular_effort = max(min(angular_velocity, 20.0), -20.0)

            if abs(heading_error) <= self.control_heading_error:
                angular_effort = 0.0
            
            self.publish_thrust_YZ(linear_velocity, angular_effort)

    def calculate_target_yaw(self, waypoint):
        dx = waypoint[0] - self.current_position[0]
        dy = waypoint[1] - self.current_position[1]

        if dx < 0:
            self.wave_control = -1.0

        target_yaw_world = math.degrees(math.atan2(dy, dx))
        target_yaw_world = self.normalize_angle(target_yaw_world + self.marvelmind_yaw)
        self.target_yaw = self.normalize_angle(target_yaw_world)
        self.get_logger().info(f"Calculated target yaw: {self.target_yaw:.2f}, Current yaw: {self.current_roll:.2f}")

    def publish_thrust_YZ(self, thrust_fwd, thrust_rot):
        self.get_logger().info(f"Rotation Thrust: {thrust_rot:.2f}")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        
        # Threshold for rotational thrust
        rot_threshold = 5.0  # Set this to a value that makes sense for your application
        
        # Clamp the forward thrust effort
        thrust_fwd = 20.0
        thrust_fwd_effort = max(min(thrust_fwd, 20.0), -20.0)
        
        cmd_vel_msg.linear.z = thrust_fwd_effort * 0.022
        if abs(thrust_rot) > rot_threshold:
            cmd_vel_msg.linear.z = 0.0
            cmd_vel_msg.linear.y = -thrust_rot * 1.9  
        
        self.cmd_vel_publisher.publish(cmd_vel_msg)
    
###############################################
###############################################
############################################### 
   
    def publish_set_yaw(self, yaw):
        self.get_logger().info(f"Publishing set yaw: {yaw:.2f} degrees")
        msg = Float32()
        msg.data = yaw
        self.set_yaw_publisher.publish(msg)     
        
    def publish_set_waypoint(self, position):
        self.get_logger().info(f"Publishing set waypoint: {position}")
        msg = Float64MultiArray()
        msg.data = [position[0], position[1]]
        self.waypoint_publisher.publish(msg)
 
        
    def calculate_distance(self, position1, position2):
        dx = position1[0] - position2[0]
        dy = position1[1] - position2[1]
        return math.sqrt(dx * dx + dy * dy)
    
          
    def run_dynamic_position(self):
        self.get_logger().info("____Dynamic Position Mode_____")

        if not hasattr(self, 'dp_reference_position'):
            # Initialize the dynamic position reference
            self.dp_reference_position = self.current_position
            self.dp_reference_yaw = self.current_roll
            self.previous_time = time.time()
            self.get_logger().info(f"Set DP reference position: {self.dp_reference_position}, DP reference yaw: {self.dp_reference_yaw:.2f} degrees")

        dx = self.dp_reference_position[0] - self.current_position[0]
        dy = self.dp_reference_position[1] - self.current_position[1]
        target_yaw_world = math.degrees(math.atan2(dy, dx))
        target_yaw_world = self.normalize_angle(target_yaw_world + self.marvelmind_yaw)
        
        self.target_yaw = self.dp_reference_yaw 
        
        heading_error = self.normalize_angle(self.target_yaw - self.current_roll)

        self.get_logger().info(f"Current Position: {self.current_position}, Reference Position: {self.dp_reference_position}, Heading Error: {heading_error:.2f}")
        self.get_logger().info(f"Current yaw: {self.current_roll:.2f} degrees, Reference yaw: {self.target_yaw:.2f} degrees")

        distance_error = self.calculate_distance(self.current_position, self.dp_reference_position)
        current_time = time.time()
        delta_time = current_time - self.previous_time
        self.previous_time = current_time  # Update previous time for next iteration
        self.get_logger().info(f"Dynamic positioning: Distance Error: {distance_error:.2f} meters, Heading Error: {heading_error:.2f}")

    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output

    def stop_asv(self):
        self.get_logger().info("Stopping ASV.")
        self.kick_back(40.0, 40.0)
        self.get_logger().info("Stopped ASV.")
        self.moving_to_waypoint = False
        
    def kick_back(self, thrust_fwd, thrust_rot):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        
        # Threshold for rotational thrust
        rot_threshold = 10.0  # Set this to a value that makes sense for your application
        
        # Clamp the forward thrust effort
        thrust_fwd = 30.0
        thrust_fwd_effort = max(min(thrust_fwd, 20.0), -20.0)
        
        cmd_vel_msg.linear.z = -thrust_fwd * 0.03
        cmd_vel_msg.linear.y = 0.0 
        
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        time.sleep(1)
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.linear.y = 0.0 
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
    def publish_twist_back(self, value_1, value_2):
        self.get_logger().info("#################### BACK ###########################")

        port_0_100_msg = Float32()
        stbd_0_100_msg = Float32()
        port_0_100_msg.data = -value_1
        stbd_0_100_msg.data = -value_2

        self.port_thrust_publisher.publish(port_0_100_msg)
        self.stbd_thrust_publisher.publish(stbd_0_100_msg)
        time.sleep(1)

    @staticmethod
    def thrust_to_pwm(thrust):
        return 1500 + thrust * 20  # Placeholder conversion

    @staticmethod
    def normalize_angle(theta):
        while (theta > 180):
            theta -= 360
        while (theta < -180):
            theta += 360
        return theta

def main(args=None):
    rclpy.init(args=args)
    simple_yaw_controller = SimpleYawController()
    executor = MultiThreadedExecutor()
    executor.add_node(simple_yaw_controller)

    try:
        simple_yaw_controller.get_logger().info('SimpleYawController node is running')
        executor.spin()
    except KeyboardInterrupt:
        simple_yaw_controller.get_logger().info('SimpleYawController node is shutting down')
    finally:
        simple_yaw_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__name__':
    main()
