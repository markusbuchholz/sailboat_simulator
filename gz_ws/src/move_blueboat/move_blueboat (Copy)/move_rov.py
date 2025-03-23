from pymavlink import mavutil
from time import sleep
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class VehicleController(Node):

    def __init__(self, connection_string='udpin:127.0.0.0:14550'):
        super().__init__('vehicle_controller')
        self.vehicle = self.create_connection(connection_string)
        self.position = {'x': 0, 'y': 0, 'z': 0}
        self.orientation = {'yaw': 0}
        self.position_received = False

        # Subscribe to Gazebo odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/model/bluerov2_heavy/odometry',
            self.odometry_callback,
            10
        )
        self.get_logger().info('Vehicle controller initialized')
        self.get_logger().info('Subscribed to /model/bluerov2_heavy/odometry')

    def create_connection(self, connection_string):
        vehicle = mavutil.mavlink_connection(connection_string)
        vehicle.wait_heartbeat()
        self.get_logger().info("Heartbeat received")
        return vehicle

    def odometry_callback(self, msg):
        self.get_logger().info('Odometry callback triggered')  # Debugging info
        self.position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        orientation_q = msg.pose.pose.orientation
        self.orientation['yaw'] = self.quaternion_to_yaw(orientation_q)
        self.position_received = True
        self.get_logger().info(f"Received position: {self.position}, Yaw: {self.orientation['yaw']}")

    @staticmethod
    def quaternion_to_yaw(q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def arm_vehicle(self):
        if not self.vehicle.motors_armed():
            self.vehicle.arducopter_arm()
            self.vehicle.motors_armed_wait()
            self.get_logger().info("Vehicle armed")

    def set_vehicle_mode(self, mode):
        mode_id = self.vehicle.mode_mapping()[mode]
        self.vehicle.set_mode(mode_id)
        self.get_logger().info(f"Vehicle mode set to {mode}")

    def send_position_request(self, x, y, z):
        type_mask = 0b0000111111111000  # Only positions
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        self.vehicle.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.vehicle.target_system,  # Target system
            self.vehicle.target_component,  # Target component
            coordinate_frame,  # Coordinate frame
            type_mask,  # Type mask
            x, y, z,  # Positions
            0, 0, 0,  # Velocities (not used)
            0, 0, 0,  # Accelerations (not used)
            0, 0  # Yaw and yaw rate (not used)
        )
        self.get_logger().info(f"Position request sent: x={x}, y={y}, z={z}")

    def perform_mission(self):
        try:
            # Arm the vehicle and set it to GUIDED mode
            self.arm_vehicle()
            self.set_vehicle_mode('GUIDED')

            # Wait for valid odometry data
            while not self.position_received:
                self.get_logger().info("Waiting for valid position data from odometry...")
                sleep(1)

            # Desired movement relative to current position
            delta_x = 2.0  # Replace with your desired movement
            delta_y = 0.0  # Replace with your desired movement
            delta_z = -2.0  # Replace with your desired movement (negative for down)

            # Calculate new position
            new_x = self.position['x'] + delta_x
            new_y = self.position['y'] + delta_y
            new_z = self.position['z'] + delta_z

            # Send a position request to move the vehicle to the new position
            self.send_position_request(new_x, new_y, new_z)

            # Wait to observe the movement
            sleep(10)

            # Fetch and print the current position after movement
            self.get_logger().info(f"Final Position: x={self.position['x']}, y={self.position['y']}, z={self.position['z']}, yaw={self.orientation['yaw']}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # Ensure the vehicle is disarmed safely
            self.disarm_vehicle()

    def disarm_vehicle(self):
        if self.vehicle.motors_armed():
            self.vehicle.arducopter_disarm()
            self.vehicle.motors_disarmed_wait()
            self.get_logger().info("Vehicle disarmed")


def main(args=None):
    rclpy.init(args=args)
    vehicle_controller = VehicleController()

    # Perform the mission
    vehicle_controller.perform_mission()

    # Keep the node alive to listen for odometry updates if needed
    rclpy.spin(vehicle_controller)

    # Clean up when done
    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
