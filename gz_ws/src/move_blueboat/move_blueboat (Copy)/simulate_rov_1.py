import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from marvelmind_ros2_msgs.msg import HedgePosition
from std_msgs.msg import Float64MultiArray
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

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

        self.callback_group = ReentrantCallbackGroup()

        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/waypoints', qos_profile=custom_qos, callback_group=self.callback_group)

        self.position_subscription = self.create_subscription(
            HedgePosition,
            '/hedgehog_pos',
            self.position_callback,
            subscriber_qos_profile,
            callback_group=self.callback_group
        )
        self.get_logger().info("Subscribed to /hedgehog_pos")

        # Define waypoints
        self.waypoints = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 7.0),
            (5.0, 7.0),
            (0.0, 7.0)
        ]
        self.current_index = 0
        self.current_position = None
        self.midpoint_tolerance = 1.0  # Tolerance range around the midpoint

        self.publish_next_waypoint()

    def publish_next_waypoint(self):
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            self.get_logger().info(f"Publishing waypoint: {waypoint}")
            msg = Float64MultiArray()
            msg.data = [waypoint[0], waypoint[1]]
            self.waypoint_publisher.publish(msg)

    def position_callback(self, msg):
        self.current_position = (msg.x_m, msg.y_m)
        self.get_logger().info(f"Current position: {self.current_position}")

        if self.current_index < len(self.waypoints) - 1:
            next_waypoint = self.waypoints[self.current_index + 1]
            if self.is_within_midpoint_range(self.current_position, self.waypoints[self.current_index], next_waypoint):
                self.current_index += 1
                self.publish_next_waypoint()

    def is_within_midpoint_range(self, current_pos, start_pos, end_pos):
        midpoint = self.calculate_midpoint(start_pos, end_pos)
        distance_to_midpoint = self.calculate_distance(midpoint, current_pos)
        self.get_logger().info(f"Distance to midpoint: {distance_to_midpoint:.2f} meters")
        return distance_to_midpoint <= self.midpoint_tolerance

    def calculate_midpoint(self, position1, position2):
        mx = (position1[0] + position2[0]) / 2.0
        my = (position1[1] + position2[1]) / 2.0
        return (mx, my)

    def calculate_distance(self, position1, position2):
        dx = position1[0] - position2[0]
        dy = position1[1] - position2[1]
        return math.sqrt(dx * dx + dy * dy)

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_publisher)

    try:
        waypoint_publisher.get_logger().info('WaypointPublisher node is running')
        executor.spin()
    except KeyboardInterrupt:
        waypoint_publisher.get_logger().info('WaypointPublisher node is shutting down')
    finally:
        waypoint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
