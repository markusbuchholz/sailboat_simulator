import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import threading

class Boat(Node):

    def __init__(self):
        super().__init__('boat')
        self.srv = self.create_service(SetBool, 'check_printing_status', self.handle_service)
        self.timer = self.create_timer(10.0, self.reset_status)
        self.printing_allowed = False
        self.get_logger().info('Boat node started')

    def handle_service(self, request, response):
        response.success = self.printing_allowed
        response.message = 'Printing allowed' if self.printing_allowed else 'Printing not allowed'
        return response

    def reset_status(self):
        self.printing_allowed = not self.printing_allowed
        self.get_logger().info(f'Boat reset service. Printing allowed: {self.printing_allowed}')

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
