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
from marvelmind_ros2_msgs.msg import HedgePosition


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


        #MPC
        self.horizon = 20
        self.Q = 0.0001
        self.R = 0.01
        self.S = 0.001


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
        self.position_tolerance = 0.75  # 0.75 meter tolerance
        
        #Core control values
        self.control_heading_error = 1.0  
        self.velo_effort_loop = 0.025
        self.yaw_effort_loop = 1.6
        self.yaw_threshold = 0.0 #5.0
        self.kick_thrust = 23.0
        self.ffw_velo_reduction = 0.6
        
        self.dp_error_yaw = 10.0
        self.dp_error_distance = 1.0
        
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
        #self.stop_asv()
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
        
        cmd_vel_msg.linear.y = -port_thrust * 3.1 
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
    def map_thrusts_to_forces(self, Tport, Tstbd, B=0.5, c=1.0):
        """
        Maps the port and starboard motor thrusts to linear and rotational forces.
        
        Parameters:
        - Tport: Thrust from the port (left) motor.
        - Tstbd: Thrust from the starboard (right) motor.
        - B: Distance between the motors (centerline-to-centerline separation).
        - c: Thruster correction factor (default is 1.0).

        Returns:
        - force_u: Linear force (forward/backward).
        - force_r: Rotational force (yaw torque).
        """
        force_u = c * (Tport + Tstbd)
        force_r = (Tstbd - Tport) / B
        
        return force_u, force_r


        
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
                self.dynamic_position = False  # True if Activate DP@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                self.linear_integral = 0.0  # Reset integral term
                self.linear_prev_error = 0.0  # Reset previous error
                self.stop_asv()
                return

            self.get_logger().info(f"Before LOS: {self.target_yaw:.2f}")
            self.calculate_target_yaw(self.target_position)  # LOS calculation
            self.get_logger().info(f"After LOS: {self.target_yaw:.2f}")

            heading_error = self.normalize_angle(self.target_yaw - self.current_roll)
            delta_time = self.timer_period
            
            linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_target, self.linear_prev_error, self.linear_integral, delta_time)
            angular_velocity = self.calculate_pid(self.angular_kPm, self.angular_kIm, self.angular_kDm, heading_error, self.prev_error, self.integral, delta_time)
          
            # Run the MPC to get the predicted thrusts
            dynamic_model = DynamicModel(initial_upsilon=np.zeros(3), initial_eta=np.array([*self.current_position, self.current_roll]))
            thrustPort, thrustStarboard = self.run_mpc(distance_to_target, heading_error, dynamic_model)
            #self.get_logger().info(f"Thrust port: {thrustPort}, Thrust star: {thrustStarboard}")
            
            # Select the final thrust values from the MPC results (typically the last in the list)
            final_thrustPort = thrustPort[-1]
            final_thrustStarboard = thrustStarboard[-1]
            
            # Map the selected thrusts to linear and rotational forces
            linear_effort, rotational_effort = self.map_thrusts_to_forces(final_thrustPort, final_thrustStarboard)
            
            
            #self.get_logger().info(f"LIN PID: {linear_velocity}, LIN MPC: {linear_effort}, ROT PID: {angular_velocity}, ROT MPC: {rotational_effort}")
        
            # Apply the rotational effort limit
            angular_effort = max(min(rotational_effort, 20.0), -20.0)
            if abs(heading_error) <= self.control_heading_error:
                angular_effort = 0.0
            
            # Publish the thrust commands
            self.publish_thrust_YZ(linear_effort, angular_effort)


    def calculate_target_yaw(self, waypoint):
        dx = waypoint[0] - self.current_position[0]
        dy = waypoint[1] - self.current_position[1]

        if dx < 0:
            self.wave_control = -1.0

        target_yaw_world = math.degrees(math.atan2(dy, dx))
        target_yaw_world = self.normalize_angle(target_yaw_world + self.marvelmind_yaw)
        self.target_yaw = self.normalize_angle(target_yaw_world)
        
        #alpha = 0.05  # Smoothing factor
        #self.target_yaw = (1 - alpha) * self.target_yaw + alpha * self.current_roll
        #self.get_logger().info(f"LOS Calculated target yaw: {self.target_yaw:.2f}, Current yaw: {self.current_roll:.2f}")
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
    
    
    def publish_thrust_YZ(self, thrust_fwd, thrust_rot):
        self.get_logger().info(f"Rotation Thrust: {thrust_rot:.2f}")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = Vector3()
        cmd_vel_msg.angular = Vector3()
        
        
        # Clamp the forward thrust effort
        thrust_fwd = 4 * thrust_fwd
        thrust_fwd_effort = max(min(thrust_fwd, 20.0), -20.0)
        
        cmd_vel_msg.linear.z = thrust_fwd_effort * self.velo_effort_loop
        if abs(thrust_rot) > self.yaw_threshold:
            cmd_vel_msg.linear.z = thrust_fwd_effort * self.velo_effort_loop * self.ffw_velo_reduction
            cmd_vel_msg.linear.y = -thrust_rot * self.yaw_effort_loop 
        
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
        
        
        # Compensate yaw if heading error is large
        #if abs(heading_error) > self.dp_error_yaw:
        #    self.publish_set_yaw(self.target_yaw)
            
        if distance_error > self.dp_error_distance:
            self.publish_set_waypoint(self.dp_reference_position)

    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output

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
