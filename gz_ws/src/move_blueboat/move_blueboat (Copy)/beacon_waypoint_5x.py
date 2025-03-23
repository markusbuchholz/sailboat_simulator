import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Float64, Float64MultiArray
from marvelmind_ros2_msgs.msg import HedgePosition
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
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

        self.subscription = self.create_subscription(
            Imu,
            '/blueboat/imu/data',
            self.listener_callback,
            custom_qos,
            callback_group=self.callback_group
        )
        # Create subscriber for the set_yaw topic
        self.set_yaw_subscription = self.create_subscription(
            Float32,
            '/set_yaw',
            self.set_yaw_callback,
            custom_qos,
            callback_group=self.callback_group
        )
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

        # Initial yaw values
        self.current_roll = None
        self.target_yaw = None

        # PID controller gains
        self.angular_kPx = 4.0  
        self.angular_kIx = 0.0  
        self.angular_kDx = 6.0 
        
        self.linear_kP = 0.5
        self.linear_kI = 0.1
        self.linear_kD = 0.05

        self.angular_kP = 4.0  
        self.angular_kI = 2.0   
        self.angular_kD = 1.0
        
        #self.angular_kP = 4.0  
        #self.angular_kI = 0.0  
        #self.angular_kD = 6.0
       
        #self.linear_kP = 1.0  
        #self.linear_kI = 0.0   
        #self.linear_kD = 0.0

        self.declare_parameter('angle_offset', 0.0)
        self.angle_offset = self.get_parameter('angle_offset').get_parameter_value().double_value

        # PID controller state
        self.integral = 0.0
        self.prev_error = 0.0
        self.linear_integral = 0.0
        self.linear_prev_error = 0.0

        # Tolerance for stopping the robot when close to the target yaw
        self.heading_tolerance = 3.0  # 5 degrees tolerance
        self.position_tolerance = 1.5  # 1 meter tolerance

        # Timer for constant checking
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.callback_group)
        self.get_logger().info("Timer started")
        
        self.current_position = None
        self.target_position = None
        self.rotating = False
        self.moving_to_waypoint = False

    def listener_callback(self, msg):
        orientation = msg.orientation
        roll, pitch, yaw = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_roll = math.degrees(roll)  # Use roll in degrees
        
    def position_callback(self, msg):
        self.current_position = (msg.x_m, msg.y_m)
        self.get_logger().info(f"current position: {self.current_position}, current orientation: {self.current_roll} ")
        
    def waypoint_callback(self, msg):
        self.target_position = (msg.data[0], msg.data[1])
        self.get_logger().info(f"Received waypoint: {self.target_position}")

        if self.current_position:
            # Calculate the relative position of the waypoint from the current position
            dx = self.target_position[0] - self.current_position[0]
            dy = self.target_position[1] - self.current_position[1]

            # Calculate the target yaw and apply the angle offset
            target_yaw_world = math.degrees(math.atan2(dy, dx))
            target_yaw_world = self.normalize_angle(target_yaw_world + 140.0)  # Adjust for 180-degree rotation
            self.get_logger().info("######################################################")
            self.target_yaw = self.normalize_angle(target_yaw_world + self.angle_offset) 
            self.get_logger().info(f"Calculated target yaw: {self.target_yaw:.2f} degrees after applying offset")
            self.rotating = True
            self.moving_to_waypoint = False
            self.get_logger().info("######################################################")

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

    def timer_callback(self):
        if self.current_roll is None:
            self.get_logger().info("Current roll not available yet.")
            return
        if self.target_yaw is None and not self.moving_to_waypoint:
            self.get_logger().info("Target yaw not available yet.")
            return

        if self.rotating:
            self.adjust_orientation()
        elif self.moving_to_waypoint:
            self.move_to_waypoint()

    def adjust_orientation(self):
        heading_error = self.normalize_angle(self.target_yaw - self.current_roll)
        self.get_logger().info(f"Current Roll: {self.current_roll:.2f} degrees, Target Yaw: {self.target_yaw:.2f} degrees, Heading Error: {heading_error:.2f} degrees")

        if abs(heading_error) <= self.heading_tolerance:
            self.get_logger().info("Target yaw achieved. Stopping thrusters.")
            self.publish_cmd_vel(0.0)  # Stop the robot
            self.integral = 0.0  # Reset integral term
            self.prev_error = 0.0  # Reset previous error
            self.rotating = False
            self.moving_to_waypoint = True  # Start moving to the waypoint
            return

        # PID calculations
        self.integral += heading_error * self.timer_period
        derivative = (heading_error - self.prev_error) / self.timer_period
        control_effort = (self.angular_kPx * heading_error) + (self.angular_kIx * self.integral) + (self.angular_kDx * derivative)
        self.prev_error = heading_error

        # Limit control effort to thrust limits and apply dead zone
        control_effort = max(min(control_effort, 25.0), -25.0)
        if -7.0 < control_effort < 7.0:
            control_effort = 0.0

        self.publish_cmd_vel(control_effort)
        self.get_logger().info(f"Control Effort: {control_effort:.2f}")
    
    def publish_thrust_ffw(self, port_thrust, stbd_thrust):
        self.get_logger().info("-----FFW-------")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        if stbd_thrust == 0:
            cmd_vel_msg.linear.y = 0.0  # performance
        else:
            #cmd_vel_msg.linear.y = -port_thrust * 3.0
            cmd_vel_msg.linear.z = port_thrust * 0.02
            #cmd_vel_msg.linear.x = stbd_thrust * 3.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
    def publish_cmd_vel(self, port_thrust, stbd_thrust=None):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        if stbd_thrust is None:
            cmd_vel_msg.linear.y = -port_thrust * 3.0  # performance
        else:
            cmd_vel_msg.linear.y = -port_thrust * 3.0
            cmd_vel_msg.linear.x = -stbd_thrust * 3.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def move_to_waypoint(self):
        self.get_logger().info("###############################################")
        self.get_logger().info("###############################################")
        
        # Calculate the relative position of the waypoint from the current position
        dx = self.target_position[0] - self.current_position[0]
        dy = self.target_position[1] - self.current_position[1]
        distance_to_target = math.sqrt(dx ** 2 + dy ** 2)
        self.get_logger().info(f"Distance to target: {distance_to_target:.2f} meters")

        if distance_to_target < self.position_tolerance:  # Threshold distance to stop
            self.get_logger().info("Waypoint reached. Stopping thrusters.")
            self.target_position = None  # Reset target position after achieving it
            self.moving_to_waypoint = False
            self.linear_integral = 0.0  # Reset integral term
            self.linear_prev_error = 0.0  # Reset previous error
            self.stop_asv()
            return

        target_yaw = math.degrees(math.atan2(dy, dx))
        heading_error = self.normalize_angle(target_yaw - self.current_roll)
        self.get_logger().info(f"Current Roll: {self.current_roll:.2f} degrees, Target Yaw: {target_yaw:.2f} degrees, Heading Error: {heading_error:.2f} degrees")

        delta_time = self.timer_period
        linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_target, self.linear_prev_error, self.linear_integral, delta_time)
        angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.prev_error, self.integral, delta_time)
        self.publish_twist(linear_velocity, angular_velocity)

    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output
    
    def stop_asv(self):
        self.get_logger().info("Stopping 1.")
        self.publish_twist(0.0, 0.0)
        #time.sleep(1)
        self.publish_twist_back(-50.0, -50.0)
        self.get_logger().info("Stopping 2.")

        
    def publish_twist_back(self, linear_x, angular_z):
        self.get_logger().info("#################### BACK ###########################")

        port_0_100_msg = Float32()
        stbd_0_100_msg = Float32()
        port_0_100_msg.data = -40.0
        stbd_0_100_msg.data = -40.0

        self.port_thrust_publisher.publish(port_0_100_msg)
        self.stbd_thrust_publisher.publish(stbd_0_100_msg)
        time.sleep(1)
        #self.port_thrust_publisher.publish(port_0_100_msg)
        #self.stbd_thrust_publisher.publish(stbd_0_100_msg)

    
    def publish_twist(self, linear_x, angular_z):
        self.get_logger().info("#################### TWIST ###########################")
        thrust_port = linear_x - angular_z
        thrust_stbd = linear_x + angular_z

        max_thrust = 10.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)

        port_thrust_msg = Float32()
        stbd_thrust_msg = Float32()
        port_thrust_msg.data = float(thrust_port)
        stbd_thrust_msg.data = float(thrust_stbd)


        pwm_port_msg = Float64()
        pwm_stbd_msg = Float64()
        pwm_port_msg.data = float(self.thrust_to_pwm(thrust_port))
        pwm_stbd_msg.data = float(self.thrust_to_pwm(thrust_stbd))


        port_float = pwm_port_msg.data
        port_0_100 = (port_float - 1100) / (8.0 * 3.0)
        stbd_float = pwm_stbd_msg.data
        stbd_0_100 = (stbd_float - 1100) / (8.0 * 3.0)

        self.get_logger().info(f"Publishing thrust: Port={port_0_100}, Starboard={stbd_0_100}")

        port_0_100_msg = Float32()
        stbd_0_100_msg = Float32()
        port_0_100_msg.data = 18.0 #float(port_0_100)
        stbd_0_100_msg.data = 18.0 #float(stbd_0_100)

        #self.port_thrust_publisher.publish(port_0_100_msg)
        #self.stbd_thrust_publisher.publish(stbd_0_100_msg)
        self.publish_thrust_ffw(25.0, 25.0)

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

if __name__ == '__main__':
    main()
