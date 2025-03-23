import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import time

class ASVController(Node):

    def __init__(self):
        super().__init__('asv_controller')

        # Custom QoS profile
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        # Subscriptions
        self.waypoint_subscription = self.create_subscription(
            Float64MultiArray, '/waypoints', self.waypoint_callback, custom_qos, callback_group=self.callback_group)
        self.odometry_subscription = self.create_subscription(
            Odometry, '/model/blueboat/odometry', self.odometry_callback, 10, callback_group=self.callback_group)

        # Publishers
        self.motor_port_publisher = self.create_publisher(
            Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', custom_qos, callback_group=self.callback_group)
        self.motor_stbd_publisher = self.create_publisher(
            Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', custom_qos, callback_group=self.callback_group)

        # State variables
        self.current_position = np.zeros(3)  # [x, y, yaw]
        self.waypoints = []
        self.current_waypoint_index = 0
        self.state = 'idle'

        # Control parameters
        self.horizon = 20
        self.Q = 0.0001
        self.R = 0.01
        self.S = 0.001
        self.max_thrust = 15.0
        self.min_thrust = -15.0

        # Angular control parameters
        self.angular_kP = 4.0
        self.angular_kI = 2.0
        self.angular_kD = 1.0

        self.angular_integral = 0.0
        self.previous_angular_error = 0.0
        self.previous_time = time.time()

        # Timer to run control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def waypoint_callback(self, msg):
        self.waypoints = np.array(msg.data).reshape(-1, 2)
        self.current_waypoint_index = 0
        self.state = 'rotate_to_waypoint'
        self.get_logger().info(f"Received waypoints: {self.waypoints}")

    # def odometry_callback(self, msg):
    #     self.current_position[0] = msg.pose.pose.position.x
    #     self.current_position[1] = msg.pose.pose.position.y
    #     orientation_q = msg.pose.pose.orientation
    #     yaw = np.arctan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
    #                      1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z))
    #     self.current_position[2] = yaw

    def control_loop(self):
        if self.state == 'idle':
            return  # Do nothing if idle
        elif self.state == 'rotate_to_waypoint':
            self.rotate_to_waypoint()
        elif self.state == 'move_to_waypoint':
            self.move_to_waypoint()
        elif self.state == 'stop_at_waypoint':
            self.stop_asv()


    def rotate_to_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
        else:
            self.state = 'idle'
            return

        # Calculate the bearing to the waypoint and the heading error
        bearing_to_waypoint = self.calculate_bearing(self.current_position[:2], waypoint)
        heading_error = self.normalize_angle(bearing_to_waypoint - self.current_position[2])

        self.get_logger().info(f"Rotating: Current Position: {self.current_position[:2]}, "
                            f"Target Waypoint: {waypoint}, Heading Error: {heading_error:.2f} radians")

        current_time = time.time()
        delta_time = current_time - self.previous_time

        # If the heading error is small enough, switch to moving towards the waypoint
        if abs(heading_error) < 0.1:
            self.state = 'move_to_waypoint'
            self.get_logger().info("Transition to state: move_to_waypoint")
        else:
            # Rotate in place: Apply angular velocity to correct the heading error
            angular_velocity = self.calculate_pid(
                self.angular_kP, self.angular_kI, self.angular_kD, heading_error,
                self.previous_angular_error, self.angular_integral, delta_time)
            # Publish thrust with zero forward motion, only angular correction
            self.publish_thrust(0.0, angular_velocity)

        self.previous_angular_error = heading_error
        self.previous_time = current_time


    def move_to_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
        else:
            self.state = 'idle'
            return

        distance_to_waypoint = self.calculate_distance(self.current_position[:2], waypoint)
        bearing_to_waypoint = self.calculate_bearing(self.current_position[:2], waypoint)
        heading_error = self.normalize_angle(bearing_to_waypoint - self.current_position[2])

        self.get_logger().info(f"Moving: Current Position: {self.current_position[:2]}, "
                               f"Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters")

        if distance_to_waypoint < 1.0:  # Stop when close to the waypoint
            self.state = 'stop_at_waypoint'
            self.stop_asv()
            self.get_logger().info("Transition to state: stop_at_waypoint")
        else:
            if abs(heading_error) > 0.1:  # If heading error is large, rotate first
                self.state = 'rotate_to_waypoint'
                self.get_logger().info("Re-adjusting heading before moving.")
            else:
                dynamic_model = DynamicModel(initial_upsilon=np.zeros(3), initial_eta=self.current_position)
                thrustPort, thrustStarboard = self.run_mpc(distance_to_waypoint, heading_error, dynamic_model)
                self.publish_thrust(thrustPort[-1], thrustStarboard[-1])

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

    def publish_thrust(self, Tport, Tstbd):
        thrust_port = max(min(Tport, self.max_thrust), self.min_thrust)
        thrust_stbd = max(min(Tstbd, self.max_thrust), self.min_thrust)

        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()

        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd

        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

    def stop_asv(self):
        self.publish_thrust(0.0, 0.0)
        self.get_logger().info("ASV Stopped")

    def rotate_to_heading(self, target_heading):
        self.target_heading = target_heading
        self.state = 'rotate_to_final_heading'

    @staticmethod
    def calculate_distance(pointA, pointB):
        return np.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)

    @staticmethod
    def calculate_bearing(pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB
        return np.arctan2(y2 - y1, x2 - x1)

    @staticmethod
    def normalize_angle(theta):
        return (theta + np.pi) % (2 * np.pi) - np.pi

    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output


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
        self.Cy = 0.001  # Coefficient for wind drag in y
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

        self.eta_dot[2] = normalize_angle(self.eta_dot[2])

        return self.eta_dot

    def update_forces(self, force_u, force_r):
        T_total = force_u
        T_diff = force_r * self.B

        self.Tport = (T_total + T_diff) / (2 * self.c)
        self.Tstbd = (T_total - T_diff) / (2 * self.c)


def apply_saturation(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def normalize_angle(angle):
    angle = np.fmod(angle + np.pi, 2 * np.pi)
    if angle < 0:
        angle += 2 * np.pi
    return angle - np.pi

def main(args=None):
    rclpy.init(args=args)
    asv_controller = ASVController()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(asv_controller, executor=executor)

    asv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
