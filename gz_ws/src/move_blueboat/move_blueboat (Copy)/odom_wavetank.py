import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float64MultiArray, Float32, Float64
from nav_msgs.msg import Odometry
import math
import time
import numpy as np
from scipy.interpolate import CubicSpline, interp1d

class WayASVController(Node):
    def __init__(self):
        super().__init__('way_asv_controller')
        
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        self.waypoint_subscription = self.create_subscription(
            Float64MultiArray, 
            '/waypoints', 
            self.waypoint_callback, 
            qos_profile=custom_qos, 
            callback_group=self.callback_group
        )

        self.odometry_subscription = self.create_subscription(
            Odometry, 
            '/model/blueboat/odometry', 
            self.odometry_callback, 
            10, 
            callback_group=self.callback_group
        )

        self.port_thrust_publisher = self.create_publisher(
            Float32, 
            '/blueboat/send_port_motor_0_100_thrust', 
            qos_profile=custom_qos, 
            callback_group=self.callback_group
        )

        self.stbd_thrust_publisher = self.create_publisher(
            Float32, 
            '/blueboat/send_stbd_motor_0_100_thrust', 
            qos_profile=custom_qos, 
            callback_group=self.callback_group
        )
        
        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10, callback_group=self.callback_group)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10, callback_group=self.callback_group)


        self.waypoints = []
        self.current_waypoint = None

        self.total_waypoints = 100
        self.waypoints_x = None
        self.waypoints_y = None

        self.linear_kP = 0.5
        self.linear_kI = 0.1
        self.linear_kD = 0.05

        self.angular_kP = 4.0  
        self.angular_kI = 2.0   
        self.angular_kD = 1.0  

        self.current_position = (0, 0)
        self.previous_position = (0, 0)  # Store the previous position
        self.current_yaw = 0.0
        self.state = 'idle'

        self.current_waypoint_index = 0

        self.target_heading = None  # Target heading for the final rotation

        self.thrust_kgf = np.array([-2.79, -2.21, -1.42, -0.82, -0.24, 0.0, 0.5, 1.17, 1.93, 2.37, 2.76, 3.57, 4.36, 5.22, 5.63])
        self.pwm_us = np.array([1110, 1188, 1292, 1370, 1448, 1500, 1552, 1604, 1656, 1682, 1708, 1760, 1812, 1864, 1900])
        self.thrust_to_pwm_interp = interp1d(self.thrust_kgf, self.pwm_us, kind='linear', fill_value="extrapolate")

        self.previous_time = time.time()
        self.linear_integral = 0.0
        self.previous_linear_error = 0.0
        self.angular_integral = 0.0
        self.previous_angular_error = 0.0

        # Timer for constant publishing
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def thrust_to_pwm(self, thrust):
        pwm = self.thrust_to_pwm_interp(thrust)
        pwm_clipped = np.clip(pwm, 1100, 1900)  # Ensure PWM stays within the valid range
        return pwm_clipped

    def interpolate_waypoints(self, waypoints, total_points):
        waypoints = np.array(waypoints)
        if len(waypoints) < 2:
            tiny_offset = np.random.normal(scale=1e-6, size=2)
            second_point = waypoints[0] + tiny_offset
            waypoints = np.vstack([waypoints, second_point])
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        t = np.linspace(0, 1, len(x))
        t_new = np.linspace(0, 1, total_points)
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)
        return x_new, y_new

    def waypoint_callback(self, msg):
        self.waypoints = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]
        self.waypoints_x, self.waypoints_y = self.interpolate_waypoints(self.waypoints, self.total_waypoints)
        self.current_waypoint_index = 0
        self.state = 'rotate_to_waypoint'
        self.get_logger().info(f"Received new waypoints: {self.waypoints}")
        self.navigate_to_waypoint()

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
        self.navigate_to_waypoint()

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

    def navigate_to_waypoint(self):
        if self.state == 'idle' or self.waypoints_x is None or self.waypoints_y is None:
            return

        if self.state == 'rotate_to_final_heading':
            final_heading_error = self.normalize_angle(self.target_heading - self.current_yaw)
            if abs(final_heading_error) < 0.1:
                self.stop_asv()
                self.state = 'idle'
                self.get_logger().info("Final heading achieved. Transitioning to idle state.")
            else:
                #self.rotate_to_target_yaw(final_heading_error)
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, final_heading_error, self.previous_angular_error, self.angular_integral, time.time() - self.previous_time)
                self.publish_twist(0.0, angular_velocity)
            return

        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
        else:
            return

        distance_to_waypoint = self.calculate_distance(self.current_position, waypoint)
        bearing_to_waypoint = self.calculate_bearing(self.current_position, waypoint)
        heading_error = self.normalize_angle(bearing_to_waypoint - self.current_yaw)

        self.get_logger().info(f"State: {self.state}, Current Position: {self.current_position}, Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters")
        self.get_logger().info(f"Heading Error: {heading_error:.2f}")

        current_time = time.time()
        delta_time = current_time - self.previous_time

        if self.state == 'rotate_to_waypoint':
            if abs(heading_error) < 0.1:
                self.state = 'move_to_waypoint'
                self.get_logger().info("Transition to state: move_to_waypoint")
            else:
                #self.rotate_to_target_yaw(heading_error)
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(0.0, angular_velocity)
        elif self.state == 'move_to_waypoint':
            if distance_to_waypoint < 1.0:
                self.state = 'stop_at_waypoint'
                self.stop_asv()
                self.get_logger().info("Transition to state: stop_at_waypoint")
            else:
                linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_waypoint, self.previous_linear_error, self.linear_integral, delta_time)
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(linear_velocity, angular_velocity)
        elif self.state == 'stop_at_waypoint':
            self.stop_asv()
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                self.state = 'rotate_to_waypoint'
                self.get_logger().info("Transition to state: rotate_to_waypoint")
            else:
                self.rotate_to_heading(self.current_yaw)  # Set the desired final heading here
                self.state = 'rotate_to_final_heading'
                self.get_logger().info("Transition to state: rotate_to_final_heading")

        self.previous_time = current_time

    def rotate_to_target_yaw(self, heading_error):
        self.get_logger().info(f"Current Yaw: {math.degrees(self.current_yaw):.2f} degrees, Heading Error: {math.degrees(heading_error):.2f} degrees")
        if abs(heading_error) > 0.1:
            if heading_error > 0:
                # Rotate counterclockwise
                port_thrust = 15.0
                stbd_thrust = -20.0
            else:
                # Rotate clockwise
                port_thrust = -20.0
                stbd_thrust = 15.0
            self.publish_thrust(port_thrust, stbd_thrust)
        else:
            self.publish_thrust(0.0, 0.0)
            self.get_logger().info("Target yaw achieved. Stopping thrusters.")

    def rotate_to_heading(self, target_heading):
        self.target_heading = target_heading
        self.state = 'rotate_to_heading'

    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output

    def publish_twist(self, linear_x, angular_z):
        thrust_port = linear_x - angular_z
        thrust_stbd = linear_x + angular_z

        max_thrust = 10.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)

        port_float = float(self.thrust_to_pwm(thrust_port))
        stbd_float = float(self.thrust_to_pwm(thrust_stbd))

        port_0_100 = (port_float - 1100) / 16.0
        stbd_0_100 = (stbd_float - 1100) / 16.0


        port_0_100_msg = Float32()
        stbd_0_100_msg = Float32()
        port_0_100_msg.data = float(port_0_100)
        stbd_0_100_msg.data = float(stbd_0_100)

        #self.port_thrust_publisher.publish(port_0_100_msg)
        #self.stbd_thrust_publisher.publish(stbd_0_100_msg)
        #self.get_logger().info(f"Publishing thrust: Port={port_0_100}, Starboard={stbd_0_100}")
        
        #to GAZEBO
        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd
        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)
        self.get_logger().info(f"Publishing thrust: Port={thrust_port}, Starboard={thrust_stbd}")







    def publish_thrust(self, port_thrust, stbd_thrust):
        port_thrust_msg = Float32()
        stbd_thrust_msg = Float32()
        port_thrust_msg.data = port_thrust
        stbd_thrust_msg.data = stbd_thrust

        self.port_thrust_publisher.publish(port_thrust_msg)
        self.stbd_thrust_publisher.publish(stbd_thrust_msg)

    def timer_callback(self):
        self.navigate_to_waypoint()

    def stop_asv(self):
        self.publish_twist(0.0, 0.0)

    @staticmethod
    def calculate_distance(pointA, pointB):
        return math.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)

    @staticmethod
    def calculate_bearing(pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB
        angle = math.atan2(y2 - y1, x2 - x1)
        return angle

    @staticmethod
    def normalize_angle(theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    way_asv_controller = WayASVController()
    executor = MultiThreadedExecutor()
    executor.add_node(way_asv_controller)

    try:
        way_asv_controller.get_logger().info('ASVController node is running')
        executor.spin()
    except KeyboardInterrupt:
        way_asv_controller.get_logger().info('ASVController node is shutting down')
    finally:
        way_asv_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
