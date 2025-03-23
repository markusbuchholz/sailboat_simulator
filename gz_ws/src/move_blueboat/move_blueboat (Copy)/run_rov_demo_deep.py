# Markus Buchholz, 2024

"""
blue@markus:~/blue_ws$ ros2 topic pub /bluerov2/attitude_request geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.7}}}" -1


"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
from time import sleep

class WaypointNavigatorNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_node')

        self.custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.waypoint_publisher = self.create_publisher(
            Float64MultiArray, '/waypoints', qos_profile=self.custom_qos, callback_group=ReentrantCallbackGroup())

        self.odom_sub = self.create_subscription(
            Odometry, 
            '/bluerov2/odometry', 
            self.odom_callback, 
            self.custom_qos, 
            callback_group=ReentrantCallbackGroup()
        )

        self.transformed_position_pub = self.create_publisher(
            PoseStamped, 
            '/bluerov2/position_request', 
            self.custom_qos, 
            callback_group=ReentrantCallbackGroup()
        )
        
        self.check_boat_subscriber = self.create_subscription(
            Bool,
            '/check_boat',
            self.check_boat_callback,
            qos_profile=self.custom_qos
        )
        
        self.attitude_publisher = self.create_publisher(
            PoseStamped, '/bluerov2/attitude_request', qos_profile=self.custom_qos, callback_group=ReentrantCallbackGroup()
        )
         ## wall, yellow, bottom [wall, yellow, -bottom], OKOKOK
        
        self.waypoints = [
            [3.0, 0.0, 0.0, False],  
            [0.0, 3.0, 0.0, False],
            [0.0, -1.5, 0.0, True],
            [0.0, 1.5, 0.0, True],
        #    [0.0, -1.5, 0.0, True],
        #    [0.0, 1.5, 0.0, True],
            [-2.5, 0.0, 0.0, False],  
            [0.0, -3.0, 0.0, False],
        ]

        self.boat_waypoints = [
            [6.0, 5.0],  
            [5.5, 3.0],
            [2.5, 3.5],  
            [3.0, 7.0], 
        ]



        self.theta_degrees = 30  
        self.translation = (0.0, 0, 0)  # Translation to apply after rotation

        self.threshold_distance = 0.2  # Distance threshold to switch to the next waypoint
        self.current_waypoint_index = 0  # Start with the first waypoint
        self.current_boat_waypoint_index = 0  # Start with the first boat waypoint
        self.initial_pos = None  # To store the initial position of the robot
        self.target_position = None  # To store the target position after transforming the waypoint
        
        self.check_boat_status = True
        self.send_orientation = False
        self.sleeping  = 13.0
        
        
    def publish_requested_attitude(self):
        attitude_msg = PoseStamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = 'base_link'
        attitude_msg.pose.position.x = 0.0
        attitude_msg.pose.position.y = 0.0
        attitude_msg.pose.position.z = 0.0
        attitude_msg.pose.orientation.x = 0.0
        attitude_msg.pose.orientation.y = 0.0
        attitude_msg.pose.orientation.z = 1.0
        attitude_msg.pose.orientation.w = 1.0

        self.attitude_publisher.publish(attitude_msg)
        self.get_logger().info("Published initial attitude request to /bluerov2/attitude_reques   t!!!!.")
        
    def check_boat_callback(self, msg):
        self.check_boat_status = msg.data
        self.get_logger().info(f'Received /check_boat status: {self.check_boat_status}')

    def odom_callback(self, msg):
        # Current odometry position
        current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

        # Set the initial position of the robot if not already set
        if self.initial_pos is None:
            self.initial_pos = current_pos
            self.update_target_position()
            self.get_logger().info(f"Initial Position set to: x={self.initial_pos[0]:.2f}, y={self.initial_pos[1]:.2f}, z={self.initial_pos[2]:.2f}")
            self.publish_transformed_position(self.target_position)
            self.publish_next_boat_waypoint()
        ###############
        ###############
        ###############
        ###############
        ###############
        ###############
        
        if (self.current_waypoint_index < len(self.waypoints)) and self.check_boat_status == False: 
            self.send_orientation = False
            # Calculate the Euclidean distance to the target position
            distance_to_target = np.sqrt(
                (current_pos[0] - self.target_position[0]) ** 2 +
                (current_pos[1] - self.target_position[1]) ** 2 +
                (current_pos[2] - self.target_position[2]) ** 2
            )
            #self.get_logger().info(f"Distance to Target: {distance_to_target:.2f}")

            # Check if the distance to the target is less than the threshold
            if distance_to_target < self.threshold_distance:
                self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached.")
                if self.send_orientation == False:
                    #self.publish_requested_attitude()
                    self.send_orientation = True

                # Update the initial position to the current position
                self.initial_pos = current_pos
                self.current_waypoint_index += 1  # Move to the next waypoint
                if self.current_waypoint_index < len(self.waypoints):
                    self.update_target_position()
                    self.get_logger().info(f"Updated Initial Position to: x={self.initial_pos[0]:.2f}, y={self.initial_pos[1]:.2f}, z={self.initial_pos[2]:.2f}")
                    #self.publish_next_boat_waypoint()  # Publish the next boat waypoint
                    self.publish_transformed_position(self.target_position)
                    ###############
                    ###############
                    ###############
                    if self.waypoints[self.current_waypoint_index][3] == False:
                        self.update_target_position()
                        self.get_logger().info(f"Updated Initial Position to: x={self.initial_pos[0]:.2f}, y={self.initial_pos[1]:.2f}, z={self.initial_pos[2]:.2f}")
                        self.publish_next_boat_waypoint()  # Publish the next boat waypoint
                        sleep(self.sleeping)
                        self.publish_transformed_position(self.target_position)
                    else:
                        self.publish_transformed_position(self.target_position)
                        self.get_logger().info("#########  DP ##############")



    def update_target_position(self):
        # Transform the current waypoint to the world frame and calculate the target position
        waypoint = self.waypoints[self.current_waypoint_index]
        transformed_waypoint = self.map_odometry_to_world([waypoint], self.theta_degrees, self.translation)[0]
        self.target_position = [
            self.initial_pos[0] + transformed_waypoint[0],
            self.initial_pos[1] + transformed_waypoint[1],
            self.initial_pos[2] + transformed_waypoint[2]
        ]

    def map_odometry_to_world(self, odometry_positions, theta_degrees, translation=(0, 0, 0)):
        theta_radians = np.radians(theta_degrees)

        cos_theta = np.cos(theta_radians)
        sin_theta = np.sin(theta_radians)

        world_positions = []

        for x_robot, y_robot, z_robot, _ in odometry_positions:
            x_world = x_robot * cos_theta - y_robot * sin_theta + translation[0]
            y_world = x_robot * sin_theta + y_robot * cos_theta + translation[1]
            z_world = z_robot + translation[2]
            world_positions.append((x_world, y_world, z_world))

        return world_positions

    def publish_transformed_position(self, final_position):
        # Create and publish PoseStamped message
        position_msg = PoseStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'world'
        position_msg.pose.position.x = final_position[0]
        position_msg.pose.position.y = final_position[1]
        position_msg.pose.position.z = final_position[2]
        position_msg.pose.orientation.w = 1.0  # Assuming no rotation in the final position

        self.transformed_position_pub.publish(position_msg)
        self.get_logger().info(
            f"Target Position Published: x={final_position[0]:.2f}, "
            f"y={final_position[1]:.2f}, z={final_position[2]:.2f}"
        )

    def publish_next_boat_waypoint(self):
        if self.current_boat_waypoint_index < len(self.boat_waypoints):
            # Prepare the next boat waypoint in 2D (x, y) for Float64MultiArray format
            waypoint = self.boat_waypoints[self.current_boat_waypoint_index]
            waypoint_msg = Float64MultiArray()
            waypoint_msg.data = [waypoint[0], waypoint[1]]  # Only include x and y coordinates

            # Publish the waypoint
            self.waypoint_publisher.publish(waypoint_msg)
            self.get_logger().info(
                f"Boat Waypoint {self.current_boat_waypoint_index + 1} Published: x={waypoint[0]:.2f}, "
                f"y={waypoint[1]:.2f}"
            )

            # Wait for a short duration before publishing the same waypoint again
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))

            # Publish the same waypoint again
            self.waypoint_publisher.publish(waypoint_msg)
            self.get_logger().info(
                f"Boat Waypoint {self.current_boat_waypoint_index + 1} Published Again: x={waypoint[0]:.2f}, "
                f"y={waypoint[1]:.2f}"
            )

            # Increment to the next waypoint index
            self.current_boat_waypoint_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
