import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ROV(Node):

    def __init__(self):
        super().__init__('rov')
        self.timer = self.create_timer(1.0, self.check_service)  # Timer to check the service status
        self.cli = self.create_client(SetBool, 'check_printing_status')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.future = None
        self.print_value = 1  # Initialize the counter
        self.service_allowed = False  # Track if printing is allowed
        self.previous_service_state = False  # Track previous service state
        self.has_printed = False  # Track if the ROV has already printed in this cycle
        self.get_logger().info('ROV node started')

    def check_service(self):
        if self.future is None or self.future.done():
            self.future = self.cli.call_async(SetBool.Request())  # Call the boat service
            self.future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.service_allowed = response.success  # Update service state based on response

            # If service is allowed, and ROV hasn't printed yet, print and set flag
            if self.service_allowed and not self.has_printed:
                self.get_logger().info(f'ROV printing: {self.print_value}')
                self.print_value += 1  # Increment the value for the next cycle
                self.has_printed = True  # Ensure it prints only once in this cycle

            # If the service is disallowed (reset by the boat), reset the printed flag
            if not self.service_allowed:
                self.has_printed = False  # Ready to print again in the next allowed cycle

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
