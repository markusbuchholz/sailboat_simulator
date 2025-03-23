import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ROV(Node):

    def __init__(self):
        super().__init__('rov')

        # Define the QoS profile
        self.custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Publisher for the waypoints
        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/waypoints', qos_profile=self.custom_qos)

        # Service client
        self.cli = self.create_client(SetBool, 'check_printing_status')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Initialize future to None
        self.future = None
        
        self.timer = self.create_timer(1.0, self.check_service)  # Timer to check the service status

        # Initialize variables
        self.waypoints = [[2.0, 2.0], [3.0, 3.0], [4.0, 4.0], [5.0, 5.0], [6.0, 6.0], [7.0, 7.0]]  # Predefined waypoints with float values
        self.waypoint_index = 0  # Index to track which waypoint to publish
        self.service_allowed = False  # Track if printing is allowed
        self.has_published = False  # Track if the waypoint has already been published in this cycle
        self.get_logger().info('ROV node started')

    def check_service(self):
        if self.future is None or self.future.done():
            self.future = self.cli.call_async(SetBool.Request())  # Call the boat service
            self.future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.service_allowed = response.success  # Update service state based on response

            # If service is allowed, and ROV hasn't published yet, publish waypoint and set flag
            if self.service_allowed and not self.has_published and self.waypoint_index < len(self.waypoints):
                # Prepare the waypoint message
                waypoint_msg = Float64MultiArray()
                waypoint_msg.data = [float(x) for x in self.waypoints[self.waypoint_index]]  # Ensure floats

                # Publish the waypoint
                self.waypoint_publisher.publish(waypoint_msg)
                
                # Log the published waypoint
                self.get_logger().info(f'Published waypoint: {self.waypoints[self.waypoint_index]}')

                # Increment waypoint index and mark as published
                self.waypoint_index += 1
                self.has_published = True  # Ensure it publishes only once in this cycle

            # Reset flag if service is disallowed
            if not self.service_allowed:
                self.has_published = False  # Ready to publish again in the next allowed cycle
                self.get_logger().info('Service disallowed. Waiting for next reset.')

        except Exception as e:
            self.get_logger().error(f'Failed to call service: {e}')

def main(args=None):
    rclpy.init(args=args)
    rov = ROV()

    # Create multithreaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(rov)

    # Spin using the multithreaded executor
    try:
        executor.spin()
    except KeyboardInterrupt:
        rov.get_logger().info('ROV node stopped by user')

    rov.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
