import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry
#from dvl_msgs.msg import DVLDR  # Ensure this matches the actual message type

import math

class SimpleYawController(Node):
    def __init__(self):
        super().__init__('simple_yaw_controller')
        
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publishers for the thrusters with custom QoS
        self.port_thrust_publisher = self.create_publisher(Float32, '/blueboat/send_port_motor_0_100_thrust', custom_qos, callback_group=self.callback_group)
        self.stbd_thrust_publisher = self.create_publisher(Float32, '/blueboat/send_stbd_motor_0_100_thrust', custom_qos, callback_group=self.callback_group)

        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10, callback_group=self.callback_group)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10, callback_group=self.callback_group)

        # Create subscriber for the DVL position to get current yaw
        # self.dvl_subscription = self.create_subscription(
        #     DVLDR,
        #     '/dvl/position',
        #     self.dvl_callback,
        #     custom_qos,
        #     callback_group=self.callback_group
        # )
        # self.get_logger().info("Subscribed to /dvl/position")


        self.odometry_subscription = self.create_subscription(
            Odometry, 
            '/model/blueboat/odometry', 
            self.odometry_callback, 
            10, 
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

        # Initial yaw values
        self.current_yaw = None
        self.target_yaw = None

        # PID controller gains
        self.angular_kP = 4.0  
        self.angular_kI = 2.0   
        self.angular_kD = 1.0

        # PID controller state
        self.integral = 0.0
        self.prev_error = 0.0

        # Timer for constant checking
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.callback_group)
        self.get_logger().info("Timer started")

    # def dvl_callback(self, msg):
    #     self.get_logger().info(f"Received DVL message: {msg}")
    #     self.current_yaw = math.radians(msg.yaw)  # Converting yaw to radians
    #     self.get_logger().info(f"Updated current yaw: {math.degrees(self.current_yaw):.2f} degrees")
        
    def odometry_callback(self, msg):
        orientation_q = [
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        self.current_yaw = yaw
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Updated current yaw: {math.degrees(self.current_yaw):.2f} degrees")
        self.get_logger().info(f"Updated current position: {self.current_position}")
    
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
        self.target_yaw = math.radians(msg.data)  # Converting target yaw to radians
        self.get_logger().info(f"Received target yaw: {math.degrees(self.target_yaw)::.2f} degrees")

    def timer_callback(self):
        if self.current_yaw is None:
            self.get_logger().info("Current yaw not available yet.")
            return
        if self.target_yaw is None:
            self.get_logger().info("Target yaw not available yet.")
            return

        heading_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        self.get_logger().info(f"Current Yaw: {math.degrees(self.current_yaw):.2f} degrees, Target Yaw: {math.degrees(self.target_yaw):.2f} degrees, Heading Error: {math.degrees(heading_error):.2f} degrees")

        # PID calculations
        self.integral += heading_error * self.timer_period
        derivative = (heading_error - self.prev_error) / self.timer_period
        control_effort = (self.angular_kP * heading_error) + (self.angular_kI * self.integral) + (self.angular_kD * derivative)
        self.prev_error = heading_error

        # Limit control effort to thrust limits and apply dead zone
        control_effort = max(min(control_effort, 25.0), -25.0)
        if -7.0 < control_effort < 7.0:
            control_effort = 0.0

        # Determine thrust values based on control effort
        port_thrust = control_effort
        stbd_thrust = -control_effort

        self.publish_thrust(port_thrust, stbd_thrust)
        self.get_logger().info(f"Control Effort: {control_effort:.2f}")

    def publish_thrust(self, port_thrust, stbd_thrust):
        port_thrust_msg = Float32()
        stbd_thrust_msg = Float32()
        port_thrust_msg.data = port_thrust
        stbd_thrust_msg.data = stbd_thrust

        self.port_thrust_publisher.publish(port_thrust_msg)
        self.stbd_thrust_publisher.publish(stbd_thrust_msg)
        
        # Gazebo
        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

        self.get_logger().info(f"Publishing thrust: Port={port_thrust}, Starboard={stbd_thrust}")

    @staticmethod
    def normalize_angle(theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi

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
