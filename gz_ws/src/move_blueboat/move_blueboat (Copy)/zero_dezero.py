import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
import numpy as np

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')

        # QoS Profiles
        self.custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscription to the Odometry topic
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/bluerov2/odometry',
            self.odometry_callback,
            self.custom_qos
        )
        self.get_logger().info("Subscribed to /bluerov2/odometry")

        # Publisher for transformed and zeroed odometry
        self.transformed_odom_pub = self.create_publisher(
            Odometry,
            '/bluerov2/odometry_transformed',
            self.custom_qos
        )

        self.initial_dvl_set = False
        self.initial_dvl = np.zeros(3)  # Initial DVL position including Z

        self.initial_pos_set = False
        self.initial_robot_world = np.zeros(3)  # Initial robot position in the world frame

        # Define the rotation angle between the robot frame and the world frame
        self.angle_degrees = 30
        self.angle_radians = np.radians(self.angle_degrees)
        self.R = self.compute_rotation_matrix(self.angle_radians)

    def compute_rotation_matrix(self, theta):
        # 2D rotation matrix extended to 3D (only rotating in the XY plane)
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        return R

    def odometry_callback(self, msg):
        # Extract robot's position from the Odometry message
        robot_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        if not self.initial_dvl_set:
            self.initial_dvl = robot_position
            self.initial_dvl_set = True
            self.get_logger().info(f'Initial DVL position set to: {self.initial_dvl}')

        if not self.initial_pos_set:
            # Set initial robot position in the world frame after zeroing
            self.initial_robot_world = np.array([0, 0, 0])
            self.initial_pos_set = True
            self.get_logger().info(f'Initial robot world position set to: {self.initial_robot_world}')

        # Calculate the required movement in the world frame
        movement_world = robot_position - self.initial_robot_world

        # Apply rotation to movement to get it in the robot frame
        movement_robot_frame = np.dot(np.linalg.inv(self.R), movement_world)

        # Compute the final DVL position by adding movement in the robot frame to the initial DVL position
        final_dvl_position = self.initial_dvl + movement_robot_frame

        # Print results
        self.get_logger().info(f'Initial Robot Position in World Frame: {self.initial_robot_world}')
        self.get_logger().info(f'Movement in World Frame: {movement_world}')
        self.get_logger().info(f'Movement in Robot Frame: {movement_robot_frame}')
        self.get_logger().info(f'Final DVL Position: {final_dvl_position}')

        # Create a new Odometry message for transformed and zeroed position
        transformed_odom_msg = Odometry()
        transformed_odom_msg.header = msg.header
        transformed_odom_msg.child_frame_id = msg.child_frame_id
        transformed_odom_msg.pose.pose.position.x = final_dvl_position[0]
        transformed_odom_msg.pose.pose.position.y = final_dvl_position[1]
        transformed_odom_msg.pose.pose.position.z = final_dvl_position[2]  # Use the final Z position
        transformed_odom_msg.pose.pose.orientation = msg.pose.pose.orientation
        transformed_odom_msg.twist = msg.twist

        # Publish the transformed and zeroed odometry
        self.transformed_odom_pub.publish(transformed_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    position_monitor = PositionMonitor()
    executor = MultiThreadedExecutor()
    executor.add_node(position_monitor)

    try:
        position_monitor.get_logger().info('PositionMonitor node is running')
        executor.spin()
    except KeyboardInterrupt:
        position_monitor.get_logger().info('PositionMonitor node is shutting down')
    finally:
        position_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
