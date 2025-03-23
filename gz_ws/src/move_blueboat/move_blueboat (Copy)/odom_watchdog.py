import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math
from nav_msgs.msg import Odometry

class WatchDog(Node):
    def __init__(self):
        super().__init__('watchdog')

        # Reentrant callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        # Subscription to /asv_pos_gps topic
        # self.asv_pos_gps_subscription = self.create_subscription(
        #     PointStamped,
        #     '/asv_pos_gps',
        #     self.asv_pos_gps_callback,
        #     10,
        #     callback_group=self.callback_group)

        # Publisher to /waypoints topic
        
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.waypoints_publisher = self.create_publisher(
            Float64MultiArray,
            '/waypoints',
            qos_profile=custom_qos,
            callback_group=self.callback_group)

        # Target position
        self.target_position = (0.0, 0.0)

        # Current position
        self.current_position = None
        
        
        self.odometry_subscription = self.create_subscription(
            Odometry, 
            '/model/blueboat/odometry', 
            self.asv_pos_odom_callback, 
            10, 
            callback_group=self.callback_group
        )

    def asv_pos_gps_callback(self, msg):
        # Extract the current position from the message
        self.current_position = (msg.point.x, msg.point.y)
        self.get_logger().info(f"Current position: {self.current_position}")

        # Compute the distance to the target position
        distance_to_target = self.compute_distance(self.current_position, self.target_position)
        self.get_logger().info(f"Distance to target: {distance_to_target:.2f} meters")

        # If the distance is less than or equal to 10 meters, publish a waypoint (3,3)
        if distance_to_target > 2.0:
            self.publish_waypoint((0.0, 0.0))
            
    def asv_pos_odom_callback(self, msg):
        # Extract the current position from the message
        
        
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Current position: {self.current_position}")

        # Compute the distance to the target position
        distance_to_target = self.compute_distance(self.current_position, self.target_position)
        self.get_logger().info(f"Distance to target: {distance_to_target:.2f} meters")

        # If the distance is less than or equal to 10 meters, publish a waypoint (3,3)
        if distance_to_target > 2.0:
            self.get_logger().info(f"DP ACTIVE")
            self.publish_waypoint((0.0, 0.0))

    def compute_distance(self, current_position, target_position):
        return math.sqrt((current_position[0] - target_position[0]) ** 2 + (current_position[1] - target_position[1]) ** 2)

    def publish_waypoint(self, waypoint):
        waypoint_msg = Float64MultiArray()
        waypoint_msg.data = [float(waypoint[0]), float(waypoint[1])]
        self.waypoints_publisher.publish(waypoint_msg)
        self.get_logger().info(f"Published waypoint: {waypoint}")

def main(args=None):
    rclpy.init(args=args)

    distance_calculator = WatchDog()

    executor = MultiThreadedExecutor()
    executor.add_node(distance_calculator)

    try:
        rclpy.spin(distance_calculator, executor)
    except KeyboardInterrupt:
        pass
    finally:
        distance_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
