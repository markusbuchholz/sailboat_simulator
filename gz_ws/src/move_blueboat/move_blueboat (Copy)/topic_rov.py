import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
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

        # Subscriber to the /check_boat topic
        self.check_boat_subscriber = self.create_subscription(
            Bool,
            '/check_boat',
            self.check_boat_callback,
            qos_profile=self.custom_qos
        )

        # Initialize variables
        self.waypoints = [[2.0, 2.0], [3.0, 3.0], [4.0, 4.0], [5.0, 5.0], [6.0, 6.0], [7.0, 7.0]]  # Predefined waypoints with float values
        self.waypoint_index = 0  # Index to track which waypoint to publish
        self.check_boat_status = True  # Initially True, so the first waypoint is sent
        self.get_logger().info('ROV node started')

        # Send the first waypoint immediately
        self.publish_waypoint()

    def check_boat_callback(self, msg):
        print("----------------------")
        self.check_boat_status = msg.data
        self.get_logger().info(f'Received /check_boat status: {self.check_boat_status}')
        if not self.check_boat_status:  # If boat allows (check_boat is False), send the next waypoint
            self.publish_waypoint()

    def publish_waypoint(self):
        if self.waypoint_index < len(self.waypoints):
            # Prepare the waypoint message
            waypoint_msg = Float64MultiArray()
            waypoint_msg.data = [float(x) for x in self.waypoints[self.waypoint_index]]  # Ensure floats

            # Publish the waypoint
            self.waypoint_publisher.publish(waypoint_msg)

            # Log the published waypoint
            self.get_logger().info(f'Published waypoint: {self.waypoints[self.waypoint_index]}')

            # Increment waypoint index
            self.waypoint_index += 1

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
