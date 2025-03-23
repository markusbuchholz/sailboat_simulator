import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class Boat(Node):

    def __init__(self):
        super().__init__('boat')

        # Define QoS profile for subscription
        self.custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Create the subscription to /waypoints topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/waypoints',
            self.waypoint_callback,
            qos_profile=self.custom_qos
        )

        # Create the publisher for /check_boat topic
        self.check_boat_publisher = self.create_publisher(Bool, '/check_boat', qos_profile=self.custom_qos)

        # Initialize variables
        self.timer = None  # Timer to reset status after receiving a waypoint
        self.get_logger().info('Boat node started')

    def waypoint_callback(self, msg):
        # Convert the data to a list for cleaner logging
        waypoint_data = list(msg.data)
        self.get_logger().info(f'Received waypoint: {waypoint_data}')

        # Publish True to /check_boat initially to indicate waypoint processing
        self.publish_check_boat_status(True)

        # Cancel any existing timer
        if self.timer is not None:
            self.timer.cancel()

        # Set a new timer to reset the /check_boat status after 5 seconds to False
        self.timer = self.create_timer(5.0, self.reset_check_boat_status)

    def publish_check_boat_status(self, status):
        # Publish the status to /check_boat only once when the timer expires
        msg = Bool()
        msg.data = status
        self.check_boat_publisher.publish(msg)
        self.get_logger().info(f'Published /check_boat status: {status}')

    def reset_check_boat_status(self):
        # Publish False to /check_boat to allow the ROV to send the next waypoint
        self.publish_check_boat_status(False)

def main(args=None):
    rclpy.init(args=args)
    boat = Boat()

    # Create multithreaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(boat)

    # Spin using the multithreaded executor
    try:
        executor.spin()
    except KeyboardInterrupt:
        boat.get_logger().info('Boat node stopped by user')

    boat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
