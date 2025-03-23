import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')

        # QoS Profiles
        self.custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        # Publisher for waypoints
        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/waypoints', qos_profile=self.custom_qos, callback_group=self.callback_group)

        # Subscription to the Odometry topic
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/bluerov2/odometry',
            self.odometry_callback,
            self.custom_qos,
            callback_group=self.callback_group
        )
        self.get_logger().info("Subscribed to /bluerov2/odometry")

        # Initialize offset variables
        self.x_offset = 0.0  # Offset for the x-coordinate
        self.y_offset = 0.0  # Offset for the y-coordinate
        self.angle_offset = 0.0  # Offset for the orientation angle (in radians)

        # Initialize position tracking
        self.initial_position = None
        self.last_published_position = None
        self.error_threshold = 0.1  # Threshold distance in meters

    def odometry_callback(self, msg):
        # Extract the current position from the odometry message
        current_position = (
            msg.pose.pose.position.x + self.x_offset,
            5.0
            #msg.pose.pose.position.y + self.y_offset
        )

        # Correct the orientation angle with offset
        current_orientation = self.correct_orientation(msg.pose.pose.orientation)
        
        self.get_logger().info(f"Adjusted position: {current_position}, Adjusted orientation angle: {math.degrees(current_orientation):.2f} degrees")

        # Set the initial position if it hasn't been set yet
        if self.initial_position is None:
            self.initial_position = current_position
            self.last_published_position = current_position
            self.publish_waypoint(current_position)
            return

        # Calculate the distance from the last published position
        distance = self.calculate_distance(current_position, self.last_published_position)
        self.get_logger().info(f"Distance from last published position: {distance:.2f} meters")

        # If the distance exceeds the threshold, publish the new position as a waypoint
        if distance > self.error_threshold:
            self.publish_waypoint(current_position)
            self.last_published_position = current_position

    def correct_orientation(self, orientation):
        """Corrects the orientation using quaternion math and applies an angle offset."""
        # Convert quaternion to euler angles
        roll, pitch, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Adjust yaw (which is the heading angle) by the offset
        adjusted_yaw = yaw + self.angle_offset

        # Normalize the angle to be within -π to π
        adjusted_yaw = self.normalize_angle(adjusted_yaw)

        return adjusted_yaw

    def euler_from_quaternion(self, x, y, z, w):
        """Converts quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)."""
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

    def normalize_angle(self, angle):
        """Normalize an angle to be within -π to π."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_waypoint(self, position):
        self.get_logger().info(f"Publishing new waypoint: {position}")
        msg = Float64MultiArray()
        msg.data = [position[0], position[1]]
        self.waypoint_publisher.publish(msg)

    def calculate_distance(self, position1, position2):
        dx = position1[0] - position2[0]
        dy = position1[1] - position2[1]
        return math.sqrt(dx * dx + dy * dy)

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