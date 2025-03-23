import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, Float32
import math
from scipy.interpolate import interp1d, CubicSpline
import time
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Vector3


class DynamicModel:
    def __init__(self, initial_upsilon=None, initial_eta=None):
        self.Tstbd = 0
        self.Tport = 0

        self.delta_x = 0
        self.delta_y = 0

        # Environmental and vehicle parameters
        self.Xu = -25
        self.Yv = 0
        self.Yr = 0
        self.Nv = 0
        self.Nr = 0
        self.X_u_dot = -2.25
        self.Y_v_dot = -23.13
        self.Y_r_dot = -1.31
        self.N_v_dot = -16.41
        self.N_r_dot = -2.79
        self.Xuu = 0
        self.Yvv = -99.99
        self.Yvr = -5.49
        self.Yrv = -5.49
        self.Yrr = -8.8
        self.Nvv = -5.49
        self.Nvr = -8.8
        self.Nrv = -8.8
        self.Nrr = -3.49

        # Vehicle parameters
        self.m = 16.0  # mass = (14.5) boat + 2 * (0.75) batteries
        self.Iz = 4.1  # moment of inertia
        self.B = 0.41  # centerline-to-centerline separation
        self.c = 1.0  # thruster correction factor

        # Environmental disturbances
        self.A = 3.0  # Wave amplitude
        self.Vw = 5.0  # Wind speed (m/s)
        self.Vc = 2.5  # Speed of the ocean current (m/s)
        self.rho_water = 1000.0  # Density of water (kg/m^3)
        self.g = 9.81  # Gravity (m/s^2)
        self.L = 2.0  # Length of the vehicle
        self.B = 2.0  # Breadth of the vehicle
        self.draft = 0.5  # Draft of the vehicle
        self.Lambda = 25000.0  # Wavelength
        self.omega_e = 0.5  # Wave frequency
        self.phi = 0  # Wave phase
        self.current_angle = np.pi / 6  # Angle of the ocean current (radians)
        self.rho_air = 1.225  # Density of air (kg/m^3)
        self.Cx = 0.001  # Coefficient for wind drag in x
        self.Cy = 0.001  # Coefficient for wind drag in motor_port_publishery
        self.Ck = 0.001  # Coefficient for yaw moment due to wind
        self.Aw = 5.0  # Area for wind
        self.Alw = 5.0  # Area for yaw wind
        self.Hlw = 2.0  # Height for yaw wind
        self.beta_w = np.pi / 4  # Wind direction relative to vehicle
        self.wave_beta = np.pi / 4  # Wave direction

        # Initialize state vectors
        self.upsilon = initial_upsilon if initial_upsilon is not None else np.zeros(3)
        self.eta = initial_eta if initial_eta is not None else np.zeros(3)
        self.upsilon_dot_last = np.zeros(3)
        self.upsilon_dot = np.zeros(3)
        self.eta_dot_last = np.zeros(3)
        self.eta_dot = np.zeros(3)

        self.M = np.array([[self.m - self.X_u_dot, 0, 0],
                           [0, self.m - self.Y_v_dot, -self.Y_r_dot],
                           [0, -self.N_v_dot, self.Iz - self.N_r_dot]])

        self.J = np.array([[np.cos(self.eta[2]), -np.sin(self.eta[2]), 0],
                           [np.sin(self.eta[2]), np.cos(self.eta[2]), 0],
                           [0, 0, 1]])
    def normalize_angle(self, angle):
        """Normalize the angle to be within the range [-pi, pi]."""
        angle = np.fmod(angle + np.pi, 2 * np.pi)
        if angle < 0:
            angle += 2 * np.pi
        return angle - np.pi

    def function_1(self, upsilon, time):
        Xu = -25
        Xuu = 0
        if np.abs(upsilon[0]) > 1.2:
            Xu = 64.55
            Xuu = -70.92

        Yv = 0.5 * (-40 * 1000 * np.abs(upsilon[1])) * \
            (1.1 + 0.0045 * (1.01 / 0.09) - 0.1 * (0.27 / 0.09) +
            0.016 * (np.power((0.27 / 0.09), 2)))
        Yr = 6 * (-3.141592 * 1000) * \
            np.sqrt(np.power(upsilon[0], 2) + np.power(upsilon[1], 2)) * 0.09 * 0.09 * 1.01
        Nv = 0.06 * (-3.141592 * 1000) * \
            np.sqrt(np.power(upsilon[0], 2) + np.power(upsilon[1], 2)) * 0.09 * 0.09 * 1.01
        Nr = 0.02 * (-3.141592 * 1000) * \
            np.sqrt(np.power(upsilon[0], 2) + np.power(upsilon[1], 2)) * 0.09 * 0.09 * 1.01 * 1.01

        Delta = np.array([self.delta_x, self.delta_y, 0])
        Delta = np.linalg.inv(self.J) @ Delta

        T = np.array([self.Tport + self.c * self.Tstbd, 0, 0.5 * self.B * (self.Tport - self.c * self.Tstbd)])

        CRB = np.array([[0, 0, -self.m * upsilon[1]],
                        [0, 0, self.m * upsilon[0]],
                        [self.m * upsilon[1], -self.m * upsilon[0], 0]])

        CA = np.array([
            [0, 0, 2 * ((self.Y_v_dot * upsilon[1]) + ((self.Y_r_dot + self.N_v_dot) / 2) * upsilon[2])],
            [0, 0, -self.X_u_dot * self.m * upsilon[0]],
            [2 * ((-self.Y_v_dot * upsilon[1]) - ((self.Y_r_dot + self.N_v_dot) / 2) * upsilon[2]), self.X_u_dot * self.m * upsilon[0], 0]
        ])

        C = CRB + CA

        Dl = np.array([[-Xu, 0, 0],
                    [0, -Yv, -Yr],
                    [0, -Nv, -Nr]])

        Dn = np.array([
            [Xuu * np.abs(upsilon[0]), 0, 0],
            [0, self.Yvv * np.abs(upsilon[1]) + self.Yvr * np.abs(upsilon[2]), self.Yrv * np.abs(upsilon[1]) + self.Yrr * np.abs(upsilon[2])],
            [0, self.Nvv * np.abs(upsilon[1]) + self.Nvr * np.abs(upsilon[2]), self.Nrv * np.abs(upsilon[1]) + self.Nrr * np.abs(upsilon[2])]
        ])

        D = Dl - Dn
        
        si = np.sin(self.omega_e * time + self.phi) * (2 * np.pi / self.Lambda) * self.A
        F_wave_x = self.rho_water * self.g * self.B * self.L * self.draft * np.cos(self.wave_beta) * si
        F_wave_y = -self.rho_water * self.g * self.B * self.L * self.draft * np.sin(self.wave_beta) * si

        uw = self.Vw * np.cos(self.beta_w - self.eta[2])
        vw = self.Vw * np.sin(self.beta_w - self.eta[2])
        Vrw = np.sqrt(uw ** 2 + vw ** 2)
        F_wind_x = 0.5 * self.rho_air * Vrw ** 2 * self.Cx * self.Aw
        F_wind_y = 0.5 * self.rho_air * Vrw ** 2 * self.Cy * self.Alw

        current_velocity_x = self.Vc * np.cos(self.current_angle)
        current_velocity_y = self.Vc * np.sin(self.current_angle)

        Fx = np.clip(T[0] - (Xu + Xuu * np.abs(upsilon[0])) + F_wave_x + F_wind_x + current_velocity_x, -1e6, 1e6)
        Fy = np.clip(T[1] - (Yv * upsilon[1] + Yr * upsilon[2]) + F_wave_y + F_wind_y + current_velocity_y, -1e6, 1e6)

        force_vector = np.array([Fx, Fy, T[2]])
        damping_force = (C @ upsilon) + (D @ upsilon)
        self.upsilon_dot = np.linalg.inv(self.M) @ (force_vector - damping_force + Delta)

        self.J = np.array([[np.cos(self.eta[2]), -np.sin(self.eta[2]), 0],
                           [np.sin(self.eta[2]), np.cos(self.eta[2]), 0],
                           [0, 0, 1]])

        self.eta_dot = self.J @ self.upsilon_dot

        self.eta_dot[2] = self.normalize_angle(self.eta_dot[2])

        return self.eta_dot

    def update_forces(self, force_u, force_r):
        T_total = force_u
        T_diff = force_r * self.B

        self.Tport = (T_total + T_diff) / (2 * self.c)
        self.Tstbd = (T_total - T_diff) / (2 * self.c)


class WayASVController(Node):
    def __init__(self):
        super().__init__('way_asv_controller')
        
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

        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10, callback_group=self.callback_group)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10, callback_group=self.callback_group)
        
        #ROS controls
        self.port_thrust_publisher = self.create_publisher(Float32, '/blueboat/send_port_motor_0_100_thrust', qos_profile=custom_qos, callback_group=self.callback_group)
        self.stbd_thrust_publisher = self.create_publisher(Float32, '/blueboat/send_stbd_motor_0_100_thrust', qos_profile=custom_qos, callback_group=self.callback_group)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/blueboat/cmd_vel", qos_profile=custom_qos, callback_group=self.callback_group)


        # Removed the navsat_subscription
        self.odometry_subscription = self.create_subscription(Odometry, '/model/blueboat/odometry', self.odometry_callback, 10, callback_group=self.callback_group)
        self.waypoint_subscription = self.create_subscription(
            Float64MultiArray,
            '/waypoints',
            self.waypoint_callback,
            subscriber_qos_profile,
            callback_group=self.callback_group
        )
        self.get_logger().info("Subscribed to /waypoints")
        self.subscription = self.create_subscription(Float32, '/bluebot/imu_compass_fused', self.fuser_callback, custom_qos, callback_group=self.callback_group)

        self.waypoints = []
        self.current_waypoint = None

        self.total_waypoints = 3
        self.waypoints_x = None
        self.waypoints_y = None

        self.horizon = 20
        self.Q = 0.0001
        self.R = 0.01
        self.S = 0.001
        self.max_thrust = 15.0
        self.min_thrust = -15.0

        self.current_position = (0, 0)
        self.current_yaw = 0.0
        self.state = 'idle'

        self.current_waypoint_index = 0

        self.target_heading = None  # Target heading for the final rotation

        self.thrust_kgf = np.array([-2.79, -2.21, -1.42, -0.82, -0.24, 0.0, 0.5, 1.17, 1.93, 2.37, 2.76, 3.57, 4.36, 5.22, 5.63])
        self.pwm_us = np.array([1110, 1188, 1292, 1370, 1448, 1500, 1552, 1604, 1656, 1682, 1708, 1760, 1812, 1864, 1900])
        self.thrust_to_pwm_interp = interp1d(self.thrust_kgf, self.pwm_us, kind='linear', fill_value="extrapolate")

        self.previous_time = time.time()
        self.angular_integral = 0.0
        self.previous_angular_error = 0.0

        # Timer for constant publishing
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        
        # Initial yaw values
        self.current_roll = None
        self.target_yaw = None
        self.fused_yaw = None
        self.marvelmind_yaw = 0.0 
        self.angle_offset = 40.0
        
        # PID controller gains
        self.angular_kPx = 4.0  
        self.angular_kIx = 0.0  
        self.angular_kDx = 6.0
        
        #Core control values
        self.control_heading_error = 1.0  
        self.velo_effort_loop = 0.025
        self.yaw_effort_loop = 1.6
        self.yaw_threshold = 0.0 #5.0
        self.kick_thrust = 23.0
        self.ffw_velo_reduction = 0.6
        
        self.dp_error_yaw = 10.0
        self.dp_error_distance = 1.0
        

    def fuser_callback(self, msg):
        self.fused_yaw = msg.data
        self.current_roll = (self.fused_yaw + self.angle_offset) % 360
        self.current_yaw = self.current_roll

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

    def waypoint_callback_sim(self, msg):
        self.waypoints = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]
        self.waypoints_x, self.waypoints_y = self.interpolate_waypoints(self.waypoints, self.total_waypoints)
        self.current_waypoint_index = 0
        self.state = 'rotate_to_waypoint'
        self.get_logger().info("########################################")
        self.get_logger().info(f"Received new waypoints: {self.waypoints}")
        self.get_logger().info(f"Received new waypoints: {self.waypoints}")
        self.get_logger().info(f"Received new waypoints: {self.waypoints}")
        self.get_logger().info("########################################")
        self.navigate_to_waypoint()
        
    def waypoint_callback(self, msg):
        #self.stop_asv()
        self.current_waypoint_index = 0
        self.state = 'rotate_to_waypoint'
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

            self.get_logger().info("Starting navigation to target waypoint")
            self.navigate_to_waypoint()

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        self.current_position = (position.x, position.y)
        #self.current_yaw = yaw

        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.point.x = float(self.current_position[0])
        pos_msg.point.y = float(self.current_position[1])

        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        if self.state == 'idle' or self.waypoints_x is None or self.waypoints_y is None:
            return

        if self.state == 'rotate_to_final_heading':
            self.stop_asv()
            self.state = 'idle'
            self.get_logger().info("Final heading achieved. Transitioning to idle state.")
            return

        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
        else:
            return

        distance_to_waypoint = self.calculate_distance(self.current_position, waypoint)
        bearing_to_waypoint = self.calculate_bearing(self.current_position, waypoint)
        heading_error = self.normalize_angle(bearing_to_waypoint - self.current_yaw)

        current_time = time.time()
        delta_time = current_time - self.previous_time
        
        self.get_logger().info(f"State: {self.state}, Current Position: {self.current_position}, Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters, Heading Error: {heading_error:.2f}")

        if self.state == 'rotate_to_waypoint':
            if abs(heading_error) < 0.1:
                self.state = 'move_to_waypoint'
                self.get_logger().info("Transition to state: move_to_waypoint")
            else:
                control_effort = self.calculate_pid(self.angular_kPx, self.angular_kDx, self.angular_kIx, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                control_effort = max(min(control_effort, 20.0), -20.0)
                if -7.0 < control_effort < 7.0:
                    control_effort = 0.0
                
                self.publish_twist_rot(0.0, control_effort)
        elif self.state == 'move_to_waypoint':
            if distance_to_waypoint < 1.0:
                self.state = 'stop_at_waypoint'
                self.stop_asv()
                self.get_logger().info("Transition to state: stop_at_waypoint")
            else:
                dynamic_model = DynamicModel(initial_upsilon=np.zeros(3), initial_eta=np.array([*self.current_position, self.current_yaw]))
                thrustPort, thrustStarboard = self.run_mpc(distance_to_waypoint, heading_error, dynamic_model)
                self.publish_thrust_YZ(thrustPort[-1], thrustStarboard[-1])
                #self.publish_twist(thrustPort[-1], thrustStarboard[-1])
        elif self.state == 'stop_at_waypoint':
            self.stop_asv()
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                self.state = 'rotate_to_waypoint'
                self.get_logger().info("Transition to state: rotate_to_waypoint")
            else:
                self.rotate_to_heading(1.0)  # Set the desired final heading here, e.g., 1.0 radians
                self.state = 'rotate_to_final_heading'
                self.get_logger().info("Transition to state: rotate_to_final_heading")

        self.previous_time = current_time
        
        
    def publish_thrust_YZ(self, thrust_fwd, thrust_rot):
        self.get_logger().info(f"Rotation Thrust: {thrust_rot:.2f}")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        
        
        # Clamp the forward thrust effort
        thrust_fwd = 20.0
        thrust_fwd_effort = max(min(thrust_fwd, 20.0), -20.0)
        
        cmd_vel_msg.linear.z = thrust_fwd_effort * self.velo_effort_loop
        if abs(thrust_rot) > self.yaw_threshold:
            cmd_vel_msg.linear.z = thrust_fwd_effort * self.velo_effort_loop * self.ffw_velo_reduction
            cmd_vel_msg.linear.y = -thrust_rot * self.yaw_effort_loop 
        
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def run_mpc(self, distance_to_waypoint, heading_error, dynamic_model):
        dt = 0.01
        t = 0.0
        horizon = self.horizon

        thrustPort = []
        thrustStarboard = []

        for _ in range(horizon):
            t += dt

            ramp_up_factor = min(1.0, t / 1.0)

            setpoint_u = distance_to_waypoint * 0.2  # Linear speed proportional to distance
            setpoint_psi = -heading_error  # Control yaw towards reducing heading error

            predicted_states = np.zeros(horizon + 1)
            predicted_states[0] = dynamic_model.upsilon[0]
            predicted_psi_states = np.zeros(horizon + 1)
            predicted_psi_states[0] = dynamic_model.upsilon[2]

            for jj in range(1, horizon + 1):
                predicted_states[jj] = predicted_states[jj - 1] + 0.1 * setpoint_u
                predicted_psi_states[jj] = predicted_psi_states[jj - 1] + 0.1 * setpoint_psi

            state_errors = predicted_states[1:] - setpoint_u
            psi_state_errors = predicted_psi_states[1:] - setpoint_psi

            Q_matrix = np.eye(horizon) * self.Q
            R_matrix = np.eye(horizon) * self.R
            S_matrix = np.eye(horizon) * self.S

            H = Q_matrix + R_matrix
            g = state_errors
            psi_g = psi_state_errors

            control_input_delta = np.linalg.solve(H, -g)
            psi_control_input_delta = np.linalg.solve(H, -psi_g)

            control_derivatives = np.zeros(horizon)
            psi_control_derivatives = np.zeros(horizon)

            control_derivatives[0] = control_input_delta[0] - 0.0
            psi_control_derivatives[0] = psi_control_input_delta[0] - 0.0

            for jj in range(1, horizon):
                control_derivatives[jj] = control_input_delta[jj] - control_input_delta[jj - 1]
                psi_control_derivatives[jj] = psi_control_input_delta[jj] - psi_control_input_delta[jj - 1]

            H += S_matrix
            g += S_matrix @ control_derivatives
            psi_g += S_matrix @ psi_control_derivatives

            control_input_delta = np.linalg.solve(H, -g)
            psi_control_input_delta = np.linalg.solve(H, -psi_g)

            control_u = ramp_up_factor * control_input_delta[0]
            control_psi = ramp_up_factor * psi_control_input_delta[0]

            dynamic_model.update_forces(control_u, control_psi)

            k1 = dynamic_model.function_1(dynamic_model.upsilon, t)
            k2 = dynamic_model.function_1(dynamic_model.upsilon + dt / 2 * k1, t + dt / 2)
            k3 = dynamic_model.function_1(dynamic_model.upsilon + dt / 2 * k2, t + dt / 2)
            k4 = dynamic_model.function_1(dynamic_model.upsilon + dt * k3, t + dt)

            dynamic_model.upsilon += dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

            thrustPort.append(dynamic_model.Tport)
            thrustStarboard.append(dynamic_model.Tstbd)

        return thrustPort, thrustStarboard

    def rotate_to_heading(self, target_heading):
        self.target_heading = target_heading
        self.state = 'rotate_to_heading'
        
    
    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output
    
    def publish_twist_rot_mpc(self, linear_x, angular_z):
        thrust_port = linear_x - angular_z
        thrust_stbd = linear_x + angular_z
        #self.get_logger().info(f"thrust_port----: {thrust_port}, thrust_stbd----: {thrust_stbd}")
        
        

        max_thrust = 20.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)


        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd
        #self.get_logger().info(f"###CHECK MPC ---- T1 values: {thrust_port}")
        #self.get_logger().info(f"###CHECK MPC ----T2 values: {thrust_stbd}")

        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)
        
    def publish_twist_rot(self, port_thrust, stbd_thrust=None):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        
        cmd_vel_msg.linear.y = -port_thrust * 3.0  
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def publish_twist(self, linear_x, angular_z):
        thrust_port = linear_x - angular_z
        thrust_stbd = linear_x + angular_z
        #self.get_logger().info(f"thrust_port----: {thrust_port}, thrust_stbd----: {thrust_stbd}")
        
        

        max_thrust = 70.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)


        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd
        #self.get_logger().info(f"###CHECK MPC ---- T1 values: {thrust_port}")
        #self.get_logger().info(f"###CHECK MPC ----T2 values: {thrust_stbd}")

        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

    def timer_callback(self):
        # Publish messages periodically
        self.navigate_to_waypoint()

    def stop_asv_mpc(self):
        self.publish_twist(0.0, 0.0)
        
    def stop_asv(self):
        self.get_logger().info("Stopping ASV.")
        self.kick_back()
        self.get_logger().info("Stopped ASV.")
        self.moving_to_waypoint = False
        
    def kick_back(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        
        # Threshold for rotational thrust
        rot_threshold = 10.0  # Set this to a value that makes sense for your application
        
    
        cmd_vel_msg.linear.z = -self.kick_thrust * 0.03
        cmd_vel_msg.linear.y = 0.0 
        
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        time.sleep(1)
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.linear.y = 0.0 
        self.cmd_vel_publisher.publish(cmd_vel_msg)

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
        while (theta > 180):
            theta -= 360
        while (theta < -180):
            theta += 360
        return theta

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