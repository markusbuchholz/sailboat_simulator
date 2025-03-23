#pip install filterpy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from filterpy.kalman import KalmanFilter

class EKFFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to DVL and IMU topics
        self.odom_sub = self.create_subscription(Odometry, '/bluerov2/odometry', self.odom_callback, qos_profile)
        self.imu_sub = self.create_subscription(Imu, '/bluerov2/imu/data', self.imu_callback, qos_profile)

        # Publisher for fused pose
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/bluerov2/fused_pose', qos_profile)

        # Initialize Kalman Filter
        self.kf = KalmanFilter(dim_x=6, dim_z=6)
        self.kf.x = np.zeros(6)  # Initial state
        self.kf.F = np.eye(6)    # State transition matrix
        self.kf.H = np.eye(6)    # Measurement function
        self.kf.P *= 1000.       # Covariance matrix
        self.kf.R = np.eye(6)    # Measurement noise
        self.kf.Q = np.eye(6) * 0.01  # Process noise

        self.last_imu_time = None
        self.last_odom_time = None
        self.stationary_threshold = 0.01  # Threshold for ZUPT
        self.stationary_count = 0

    def odom_callback(self, msg):
        # Extract DVL position and velocity
        dvl_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        dvl_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

        measurement = np.hstack((dvl_pos, dvl_vel))
        self.kf.update(measurement)
        self.last_odom_time = self.get_clock().now()

        # Publish the fused pose
        self.publish_fused_pose()

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            self.kf.F = np.eye(6) + np.array([[0, 0, 0, dt, 0, 0],
                                              [0, 0, 0, 0, dt, 0],
                                              [0, 0, 0, 0, 0, dt],
                                              [0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0]])

            # Zero Velocity Update (ZUPT)
            lin_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            if np.linalg.norm(lin_acc) < self.stationary_threshold:
                self.stationary_count += 1
            else:
                self.stationary_count = 0

            if self.stationary_count >= 10:
                self.kf.x[3:] = 0

            self.kf.predict()

        self.last_imu_time = current_time

    def publish_fused_pose(self):
        fused_pose_msg = PoseWithCovarianceStamped()
        fused_pose_msg.header.stamp = self.get_clock().now().to_msg()
        fused_pose_msg.header.frame_id = 'map'

        fused_pose_msg.pose.pose.position.x = self.kf.x[0]
        fused_pose_msg.pose.pose.position.y = self.kf.x[1]
        fused_pose_msg.pose.pose.position.z = self.kf.x[2]

        # Fill in the covariance matrix if needed
        fused_pose_msg.pose.covariance = np.hstack((self.kf.P[:3, :3], np.zeros((3, 3)),
                                                    np.zeros((3, 3)), np.zeros((3, 3)))).flatten().tolist()

        self.pose_pub.publish(fused_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
