import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import time
import math

class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        # Custom QoS Profile with default settings to match the publisher
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            Odometry,
            '/model/blueboat/odometry',
            self.odom_callback,
            qos_profile=custom_qos,
            callback_group=self.callback_group)

        self.waypoint_subscription = self.create_subscription(
            Float64MultiArray,
            '/waypoints',
            self.waypoint_callback,
            qos_profile=custom_qos,
            callback_group=self.callback_group)

        self.current_position = None
        self.target_waypoints = []
        self.current_waypoint_index = 0
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.callback_group)

        # Connect to the vehicle
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14570')
        self.master.wait_heartbeat()
        self.arm_vehicle()

    def arm_vehicle(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        self.get_logger().info("Attempting to arm the vehicle...")
        timeout = time.time() + 10  # 10 second timeout for arming
        armed = False

        while not armed and time.time() < timeout:
            message = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if message:
                armed = message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                if armed:
                    self.get_logger().info('Vehicle is armed!')
                    break

        if not armed:
            self.get_logger().error('Failed to arm the vehicle. Check pre-arm conditions and messages.')

    def set_mode(self, mode_name):
        if mode_name not in self.master.mode_mapping():
            self.get_logger().error(f"Mode {mode_name} not found in mode mapping. Exiting...")
            return
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.set_mode(mode_id)
        self.get_logger().info(f"Mode set to {mode_name}.")

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Current position updated to: {self.current_position}")

    def waypoint_callback(self, msg):
        self.target_waypoints = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]
        self.current_waypoint_index = 0
        self.set_mode('GUIDED')
        self.get_logger().info(f"Received waypoints: {self.target_waypoints}")

    def timer_callback(self):
        if self.current_position is None or not self.target_waypoints:
            return

        target_waypoint = self.target_waypoints[self.current_waypoint_index]
        distance_to_waypoint = self.distance(self.current_position, target_waypoint)

        if distance_to_waypoint < 0.5:  # threshold distance to consider waypoint reached
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.target_waypoints):
                self.get_logger().info("All waypoints reached.")
                self.set_mode('LOITER')
                self.target_waypoints = []
                return
            target_waypoint = self.target_waypoints[self.current_waypoint_index]

        self.move_towards(target_waypoint)

    def distance(self, position1, position2):
        return math.sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2)

    def move_towards(self, target_waypoint):
        self.send_target_position(target_waypoint[0], target_waypoint[1])
        self.get_logger().info(f"Moving towards waypoint: {target_waypoint}")

    def send_target_position(self, x, y):
        # Send the target position to the vehicle
        self.master.mav.set_position_target_local_ned_send(
            0,                         # time_boot_ms
            self.master.target_system, # target_system
            self.master.target_component, # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,        # type_mask (only positions enabled)
            x,                         # X Position in meters
            y,                         # Y Position in meters
            0,                         # Z Position in meters
            0, 0, 0,                   # X, Y, Z velocity in m/s (not used)
            0, 0, 0,                   # afx, afy, afz acceleration (not used)
            0, 0)                      # yaw, yaw rate (not used)
        self.get_logger().info(f"Sent target position: x={x}, y={y}")

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_follower)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        waypoint_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
