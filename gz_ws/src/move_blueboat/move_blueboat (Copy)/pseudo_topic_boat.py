import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64MultiArray
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

        # Create the service that ROV will use to check if printing is allowed
        self.srv = self.create_service(SetBool, 'check_printing_status', self.handle_service)

        self.printing_allowed = True  # Allow ROV to publish initially
        self.timer = None  # Timer to reset service
        self.get_logger().info('Boat node started')

    def waypoint_callback(self, msg):
        # Convert the data to a list for cleaner logging
        waypoint_data = list(msg.data)
        self.get_logger().info(f'Received waypoint: {waypoint_data}')

        # Disallow further publishing immediately after receiving a waypoint
        self.printing_allowed = False

        # Cancel any existing timer
        if self.timer is not None:
            self.timer.cancel()

        # Set a new timer to reset the service status after 5 seconds
        self.get_logger().info(f'Service will be reset in 5 seconds.')
        self.timer = self.create_timer(1.0, self.reset_status)


    def handle_service(self, request, response):
        # Handle ROV's request to check if publishing is allowed
        response.success = self.printing_allowed
        response.message = 'Printing allowed' if self.printing_allowed else 'Printing not allowed'
        return response

    def reset_status(self):
        # Reset the service status to allow publishing
        self.printing_allowed = True
        self.get_logger().info(f'Service reset. Printing allowed: {self.printing_allowed}')

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
